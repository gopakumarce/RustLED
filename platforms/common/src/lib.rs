const NANOSECS: u64 = 1_000_000_000;

pub struct LedSpec {
    pub t1_ns: usize,
    pub t2_ns: usize,
    pub t3_ns: usize,
}

pub struct CalibrationResult {
    pub divisor: u16,
    pub t1_pulses: usize,
    pub t2_pulses: usize,
    pub t3_pulses: usize,
}

pub struct ClockDivisors {
    pub divisor: u32,
    pub numerator: u32,
    pub denominator: u32,
}

// Copied off esp api i2s_ll_tx_set_mclk(), just for debugging, to see what values
// are eventually set in esp
pub fn calculate_divisors(
    sclk: u32,
    mclk: u32,
    mut mclk_div: u32,
    divider_max: u32,
) -> ClockDivisors {
    let mut ma: u32;
    let mut mb: u32;
    let mut denominator: u32 = 1;
    let mut numerator: u32 = 0;

    let freq_diff: u32 = sclk.abs_diff(mclk * mclk_div) as u32;
    if freq_diff == 0 {
        return ClockDivisors {
            divisor: mclk_div,
            numerator,
            denominator,
        };
    }
    let decimal: f64 = freq_diff as f64 / mclk as f64;
    // Carry bit if the decimal is greater than 1.0 - 1.0 / (63.0 * 2) = 125.0 / 126.0
    if decimal > 125.0 / 126.0 {
        mclk_div += 1;
        return ClockDivisors {
            divisor: mclk_div,
            numerator,
            denominator,
        };
    }
    let mut min: u32 = !0;
    for a in 2..=divider_max {
        let b = ((a as f64 * (freq_diff as f64 / mclk as f64)) as f64 + 0.5) as u32;
        ma = freq_diff * a;
        mb = mclk * b;
        if ma == mb {
            denominator = a;
            numerator = b;
            return ClockDivisors {
                divisor: mclk_div,
                numerator,
                denominator,
            };
        }
        if (mb.abs_diff(ma) - ma) < min {
            denominator = a;
            numerator = b;
            min = mb.abs_diff(ma);
        }
    }

    ClockDivisors {
        divisor: mclk_div,
        numerator,
        denominator,
    }
}

// The led spec is basically copied from FastLED code, its such that a zero is
// represented by signal that is high for t1 nsecs and low for t2+t3 nsecs.
// And a one is represented as high for t1+t2 nsecs and low for t3 nsecs.
// There are no t1/t2/t3 in the manufacturer's specs, the manufacturers just
// specify the high/low times for zero and one. The FastLED folks have split
// it into three pieces to make it a lil more "scaleable" as discussed below
//
// Lets say the base clock of the esp is B (like 160Mhz). So in the spec, lets
// say t1/t2/t3 ns is T1/T2/T3 ticks of the base clock. Remember that a zero or
// one of the LED needs t1+t2+t3 nsecs == T1+T2+T3 ticks of the base clock. So
// we need to send T1+T2+T3 bits on the I2S data pin to make it look like ONE
// bit for the LED. So obviously, a large T1+T2+T2 means that we need to store
// too many bits in dma memory, and also too many interrupts to handle once the
// dma is succesful (to load the next set of bits etc..). So the goal is how can
// we reduce the number of clock bits needed to represent one LED bit. And we can
// do that by scaling down the clock. ESP provides a setting by which the base
// clock can be scaled down as B/(N + b/a) where (N + b/a) is a value with three
// configurable parameters N, b and a.
//
// So for us the task is that we need to scale down T1, T2 and T3 into values
// say K1, K3 K3 where K1 = T1/(N + b/a) and K2 = T2/(N + b/a) and K3 = T3/(N + b/a)
// So each of the three pieces are scaled down, and hence the total sum is
// also scaled down. But now the challenge is in finding these values for N, b and a
//
// We are again copying what FastLED folks did. So their approach is that lets say
// we are willing to trade off on certain amount of "precision". For example,
// K1 times (N + b/a) might not be equal to T1, but rather T1 plus or minus some
// "precision/tolerance" number of clock ticks say p1. And similarly p2, and p3 for
// T2 and T3. And p1, p2 and p3 might all not be the same, i.e the difference
// when scaling back each of those three pieces might be different, but within the
// tolerance levels.
//
// So given that is the approach, we basically start to find a divisor for T1, T2, T3
// such that the remainder is within some "precision" we define. All the manufactures
// talk about the tolerance in their specs, so we can translate that to some number
// of base clock ticks. And we find the largest divisor that is within the precision
// tolerance
pub fn calibrate_clocks(spec: &LedSpec, base_clock: u64, precision: usize) -> CalibrationResult {
    let t1 = (spec.t1_ns as u64 * base_clock) / NANOSECS;
    let t2 = (spec.t2_ns as u64 * base_clock) / NANOSECS;
    let t3 = (spec.t3_ns as u64 * base_clock) / NANOSECS;

    let min = std::cmp::min(t3, std::cmp::min(t1, t2));
    let mut divisor: u64 = 1;
    for d in (1..=min).rev() {
        let mut stop = false;
        for p in 0..precision as u64 {
            if t1 % d <= p && t2 % d <= p && t3 % d <= p {
                divisor = d as u64;
                stop = true;
                break;
            }
        }
        if stop {
            break;
        }
    }

    CalibrationResult {
        divisor: divisor as u16,
        t1_pulses: (t1 / divisor) as usize,
        t2_pulses: (t2 / divisor) as usize,
        t3_pulses: (t3 / divisor) as usize,
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
    const I2S_LL_MCLK_DIVIDER_MAX: u32 = 63;
    const BASE_CLOCK: u64 = 160 * 1_000_000;
    const PRECISION_TICKS: usize = 10;

    #[test]
    fn test_clock() {
        let spec = LedSpec {
            t1_ns: 250,
            t2_ns: 625,
            t3_ns: 375,
        };
        let result = calibrate_clocks(&spec, BASE_CLOCK, PRECISION_TICKS);
        let divisors = calculate_divisors(
            BASE_CLOCK as u32,
            BASE_CLOCK as u32 / result.divisor as u32,
            result.divisor as u32,
            I2S_LL_MCLK_DIVIDER_MAX,
        );
        assert_eq!(result.divisor, 20);
        assert_eq!(divisors.divisor, 20);
        assert_eq!(divisors.numerator, 0);
        assert_eq!(divisors.denominator, 1);
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
}
