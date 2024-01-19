const MICROSECS: usize = 1_000_000;
const NANOSECS: usize = MICROSECS * 1_000;
const I2S_MAX_PULSE_PER_BIT: usize = 20;

// For example WS2811 specs will be
// t1_ns: 320
// t2_ns: 320
// t3_ns: 641
pub struct LedSpec {
    pub t1_ns: usize,
    pub t2_ns: usize,
    pub t3_ns: usize,
}

#[derive(Debug)]
pub struct ClockDivisors {
    pub divisor: usize,
    pub numerator: usize,
    pub denominator: usize,
    pub t1_ticks: usize,
    pub t2_ticks: usize,
    pub t3_ticks: usize,
}

fn pgcd(mut smallest: usize, precision: usize, a: usize, b: usize, c: usize) -> usize {
    while smallest > 0 {
        if a % smallest <= precision && b % smallest <= precision && c % smallest <= precision {
            return smallest;
        }
        smallest -= 1;
    }
    return 1;
}

// This is basically copied off initBitPatterns() in FastLED clockless_i2s_esp32.h
// The basic logic is straight forward as below
// 1. Given the nanoseconds - convert it into CPU ticks
// 2. Divide the cpu ticks by the greatest common denominator (gcd) so that
//    we find as small a set of ticks as possible
// 3. Given that we have a timing requirement of t1_ns+t2_ns+t3ns, that means whatever
//    clock we use will "at least" have a frequency of 1/(t1_ns + t2_ns + t3_ns)
//    So, after doing the least possible scale, that frequency will be multiplied by the scale
// 4. Now we know the i2s_clock frequency, our job is to bring it down to the frequency
//    mentioned in the step above. A ratio of the frequencies gives us the "divisor"
// 5. Now its about fine tuning it such that we use the (i2_clock) / (divisor + b/a) formula
//    which may let us get closer to the actual required frequency by manpulating b and a
// 6. So we take the diff of expected frequency and just the i2s_clock/divisor frequency and
//    try to get to that diff value using b/a
//
// NOTE1: First of all, not sure why we need to do the initial steps using the cpu clock
// frequency, we can use one clock, the i2s_clock for that math also ? Basically regardless
// of what clock is used for that math, the goal is just to reduce the numbers by their GCD
//
// NOTE2: The act of taking the "diff" in step 5 above and getting to it usig b/a, that doesnt
// sound a very convincing math ?
pub fn scale_clocks(spec: &LedSpec, cpu_clock: usize, i2s_clock: usize) -> ClockDivisors {
    // Convert nano seconds into CPU ticks. Note that are clocks are in Hertz
    let cpu_mhz = cpu_clock / MICROSECS;
    let t1 = (spec.t1_ns * cpu_mhz + 999) / 1000;
    let t2 = (spec.t2_ns * cpu_mhz + 999) / 1000;
    let t3 = (spec.t3_ns * cpu_mhz + 999) / 1000;

    let t1_ns = (t1 * 1000) / cpu_mhz;
    let t2_ns = (t2 * 1000) / cpu_mhz;
    let t3_ns = (t3 * 100) / cpu_mhz;

    let smallest = std::cmp::min(t3, std::cmp::min(t1, t2));
    let mut precision = 0;
    let mut pgc;
    let mut t1_ticks;
    let mut t2_ticks;
    let mut t3_ticks;
    loop {
        pgc = pgcd(smallest, precision, t1, t2, t3);
        t1_ticks = t1 / pgc;
        t2_ticks = t2 / pgc;
        t3_ticks = t3 / pgc;
        if pgc != 1 && (t1_ticks + t2_ticks + t3_ticks) <= I2S_MAX_PULSE_PER_BIT {
            break;
        }
        precision += 1;
    }

    let pulses_per_bit = (1 / pgc + t2 / pgc + t3 / pgc) as f32;
    let nsecs = (t1_ns + t2_ns + t3_ns) as f32;
    let freq = (pulses_per_bit / nsecs) * NANOSECS as f32;

    let precision = (1 as f32) / (63 as f32);
    let prec_by2: f32 = precision / (2 as f32);
    let divisor_float: f32 = i2s_clock as f32 / freq;
    let mut divisor: usize = (i2s_clock as f32 / freq) as usize;
    let diff: f32 = divisor_float - divisor as f32;
    let mut numerator = 0;
    let mut denominator = 1;

    for a in 1..64 {
        let mut b = 0;
        while b < a {
            if f32::abs(diff - (b as f32) / (a as f32)) <= prec_by2 {
                break;
            }
            b += 1;
        }
        let cur = f32::abs(diff - (b as f32) / (a as f32));
        if cur == 0 as f32 {
            numerator = b;
            denominator = a;
            break;
        }
        let prev = f32::abs(diff - (numerator as f32) / (denominator as f32));
        if cur < prec_by2 && cur < prev {
            numerator = b;
            denominator = a;
        }
    }

    if numerator == denominator {
        numerator = 0;
        denominator = 1;
        divisor += 1;
    }
    ClockDivisors {
        divisor,
        numerator,
        denominator,
        t1_ticks,
        t2_ticks,
        t3_ticks,
    }
}

pub fn transpose32(pixels: &[u8], bits: &mut [u8]) {
    transpose8r_s32(&pixels[0..], 1, 4, &mut bits[0..]);
    transpose8r_s32(&pixels[8..], 1, 4, &mut bits[1..]);
    transpose8r_s32(&pixels[16..], 1, 4, &mut bits[2..]);
    transpose8r_s32(&pixels[24..], 1, 4, &mut bits[3..]);
}

fn transpose8r_s32(a: &[u8], m: usize, n: usize, b: &mut [u8]) {
    let mut x: u32;
    let mut y: u32;
    let mut t: u32;

    x = (a[0] as u32) << 24 | (a[m] as u32) << 16 | (a[2 * m] as u32) << 8 | a[3 * m] as u32;
    y = (a[4 * m] as u32) << 24
        | (a[5 * m] as u32) << 16
        | (a[6 * m] as u32) << 8
        | a[7 * m] as u32;

    t = (x ^ (x >> 7)) & 0x00AA00AA;
    x = x ^ t ^ (t << 7);
    t = (y ^ (y >> 7)) & 0x00AA00AA;
    y = y ^ t ^ (t << 7);

    t = (x ^ (x >> 14)) & 0x0000CCCC;
    x = x ^ t ^ (t << 14);
    t = (y ^ (y >> 14)) & 0x0000CCCC;
    y = y ^ t ^ (t << 14);

    t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F);
    y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F);
    x = t;

    b[0] = (x >> 24) as u8;
    b[n] = (x >> 16) as u8;
    b[2 * n] = (x >> 8) as u8;
    b[3 * n] = x as u8;
    b[4 * n] = (y >> 24) as u8;
    b[5 * n] = (y >> 16) as u8;
    b[6 * n] = (y >> 8) as u8;
    b[7 * n] = y as u8;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clock() {
        let cpu_clock = 240 * 1_000_000;
        let i2s_clock = 80 * 1_000_000;
        let spec = LedSpec {
            t1_ns: 320,
            t2_ns: 320,
            t3_ns: 641,
        };
        let result = scale_clocks(&spec, cpu_clock, i2s_clock);
        println!("{:?}", result);
        assert_eq!(result.divisor, 25);
        assert_eq!(result.numerator, 31);
        assert_eq!(result.denominator, 50);
        assert_eq!(result.t1_ticks, 1);
        assert_eq!(result.t2_ticks, 1);
        assert_eq!(result.t3_ticks, 2);
    }

    #[test]
    fn test_transpose() {
        let sample = vec![0xaau8; 32];
        let mut bits = vec![0u8; 32];
        transpose32(&sample, &mut bits);
        for i in 0..32 {
            print!("{:#04x} ", bits[i])
        }
        println!();

        let sample = vec![0xccu8; 32];
        let mut bits = vec![0u8; 32];
        transpose32(&sample, &mut bits);
        for i in 0..32 {
            print!("{:#04x} ", bits[i])
        }
        println!();
    }

    // This test basically takes the ColorPallette example from
    // FastLED examples, picks the first palette, gets it R/G/B
    // values by dumping them inside fillBuffer inside FastLED code
    // and then also gets the transposed values / indices from
    // dumping inside FastLED code and ensures that the same logic
    // translated here produces the same values
    #[test]
    fn test_transpose_rgb() {
        let colors = [
            [
                0x00000000, 0x00000001, 0x00000003, 0x00000004, 0x00000005, 0x00000007, 0x00000008,
                0x0000000a, 0x0000000b, 0x0000000d, 0x0000000e, 0x0000000f, 0x00000011, 0x00000012,
                0x00000013, 0x00000015, 0x00000016, 0x00000018, 0x00000019, 0x0000001b, 0x0000001c,
                0x0000001e, 0x0000001f, 0x00000020, 0x00000022, 0x00000023, 0x00000024, 0x00000026,
                0x00000027, 0x00000029, 0x0000002a, 0x0000002b, 0x0000002c, 0x0000002b, 0x00000029,
                0x00000028, 0x00000026, 0x00000025, 0x00000024, 0x00000022, 0x00000021, 0x0000001f,
                0x0000001e, 0x0000001c, 0x00000019, 0x00000016, 0x00000013, 0x00000010, 0x0000000e,
                0x0000000b,
            ],
            [
                0x00000040, 0x0000003e, 0x0000003c, 0x00000039, 0x00000037, 0x00000036, 0x00000034,
                0x00000032, 0x0000002f, 0x0000002d, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b,
                0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b, 0x0000002b,
                0x0000002b, 0x00000027, 0x00000023, 0x0000001e, 0x0000001a, 0x00000016, 0x00000013,
                0x0000000e, 0x0000000a, 0x00000006, 0x00000002, 0x00000000, 0x00000000, 0x00000000,
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                0x00000000,
            ],
            [
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
                0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000002, 0x00000004,
                0x00000006, 0x00000008, 0x0000000a, 0x0000000b, 0x0000000d, 0x0000000f, 0x00000011,
                0x00000013, 0x00000016, 0x0000001a, 0x0000001e, 0x00000022, 0x00000025, 0x00000029,
                0x0000002d,
            ],
        ];

        let mut next_seq = 0;
        let sequences = [
            90, 70, 100, 110, 120, 130, 140, 60, 70, 100, 110, 120, 130, 50, 100, 110, 120, 150,
            50, 70, 100, 110, 130, 140, 150, 50, 60, 70, 100, 110, 130, 140, 40, 100, 110, 130, 40,
            60, 100, 110, 140, 40, 60, 70, 100, 120, 130, 140, 150, 40, 50, 70, 100, 120, 130, 150,
            40, 50, 60, 100, 120, 140, 150, 40, 50, 60, 70, 100, 120, 140, 150, 30, 70, 100, 120,
            140, 150, 30, 60, 100, 120, 140, 150, 30, 60, 70, 100, 120, 140, 150, 30, 50, 70, 100,
            120, 140, 150, 30, 50, 60, 100, 120, 140, 150, 30, 40, 100, 120, 140, 150, 30, 40, 70,
            100, 120, 140, 150, 30, 40, 60, 70, 100, 120, 140, 150, 30, 40, 50, 100, 120, 140, 150,
            30, 40, 50, 60, 100, 120, 140, 150, 30, 40, 50, 60, 70, 100, 130, 140, 150, 20, 100,
            140, 150, 20, 60, 110, 120, 130, 140, 20, 60, 70, 110, 120, 140, 20, 50, 110, 130, 140,
            20, 50, 60, 110, 140, 150, 20, 50, 60, 70, 120, 130, 140, 20, 40, 70, 120, 140, 20, 40,
            60, 130, 140, 20, 40, 60, 70, 140, 20, 40, 50, 20, 40, 60, 70, 220, 20, 40, 70, 210,
            20, 40, 210, 220, 20, 50, 60, 200, 20, 50, 70, 200, 220, 20, 50, 200, 220, 230, 20, 60,
            200, 210, 230, 20, 70, 200, 210, 220, 230, 30, 40, 50, 60, 70, 190, 230, 30, 40, 50,
            60, 190, 220, 230, 30, 40, 50, 190, 210, 220, 30, 40, 70, 190, 200, 220, 30, 50, 60,
            190, 200, 210, 220, 30, 60, 70, 180, 220, 30, 180, 210, 230, 40, 50, 60, 180, 200, 230,
            40, 60, 70, 180, 200, 210, 230,
        ];
        assert_eq!(colors[0].len(), colors[1].len());
        assert_eq!(colors[0].len(), colors[2].len());

        for color in 0..colors[0].len() {
            for channel in 0..3 {
                let mut g_pixel_row: [u8; 32] = [0; 32];
                let mut g_pixel_bits: [u8; 32] = [0; 32];
                g_pixel_row[23] = colors[channel][color];
                transpose32(&g_pixel_row[..], &mut g_pixel_bits[..]);
                for bitnum in 0..8 {
                    let offset = bitnum * 4;
                    // Think of pixel_words as a word which is in blocks of four bytes
                    let byte0 = g_pixel_bits[offset];
                    let byte1 = g_pixel_bits[offset + 1];
                    let byte2 = g_pixel_bits[offset + 2];
                    let byte3 = g_pixel_bits[offset + 3];
                    // reverse the bits
                    let bits = (u32::from(byte0) << 24)
                        | (u32::from(byte1) << 16)
                        | (u32::from(byte2) << 8)
                        | u32::from(byte3);
                    let g_pulses_per_bit = 10;
                    let index = bitnum * g_pulses_per_bit + channel * 8 * g_pulses_per_bit;
                    if bits != 0 {
                        assert_eq!(bits, 0x100);
                        assert_eq!(index, sequences[next_seq]);
                        next_seq += 1;
                    }
                }
            }
        }
    }
}
