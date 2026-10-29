/// Linear-interpolation resampler for interleaved stereo, with state that carries across calls so
/// a stream cut into chunks comes out as it would uncut.
///
/// Linear interpolation aliases when it lowers the rate and rolls off the top octave when it
/// raises it. The input here is a 16-bit iPod stream going to speakers, and the common case, 44.1
/// or 48 kHz to a device at one of those, is either a straight copy or a ratio of 1.088.
///
/// Positions are integers in units of 1/`out_rate` of an input frame, so no error accumulates over
/// a long stream.
#[derive(Debug, Clone)]
pub struct Resampler {
    in_rate: u32,
    out_rate: u32,
    /// Position of the next output frame, counted from the last input frame consumed.
    pos: u64,
    prev: [f32; 2],
    primed: bool,
}

impl Resampler {
    pub fn new(in_rate: u32, out_rate: u32) -> Self {
        Self {
            in_rate,
            out_rate: out_rate.max(1),
            pos: 0,
            prev: [0.0; 2],
            primed: false,
        }
    }

    pub fn in_rate(&self) -> u32 {
        self.in_rate
    }

    /// Append the output for interleaved stereo `input` to `out`.
    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        let n = input.len() / 2;
        if n == 0 {
            return;
        }
        if self.in_rate == self.out_rate {
            out.extend_from_slice(&input[..n * 2]);
            return;
        }
        let mut input = &input[..n * 2];
        if !self.primed {
            self.prev = [input[0], input[1]];
            self.primed = true;
            self.pos = 0;
            input = &input[2..];
        }
        self.walk(input, out);
    }

    // The sequence is `prev` followed by the input: index 0 is `prev`, index i is input frame
    // i - 1. An output at position p needs indices floor(p) and floor(p) + 1, so it can be
    // produced once floor(p) < n.
    fn walk(&mut self, input: &[f32], out: &mut Vec<f32>) {
        let n = input.len() / 2;
        let at = |i: usize| -> [f32; 2] {
            if i == 0 {
                self.prev
            } else {
                [input[(i - 1) * 2], input[(i - 1) * 2 + 1]]
            }
        };
        let den = u64::from(self.out_rate);
        let mut p = self.pos;
        while p / den < n as u64 {
            let i = (p / den) as usize;
            let f = (p % den) as f32 / den as f32;
            let (a, b) = (at(i), at(i + 1));
            out.push(a[0] + (b[0] - a[0]) * f);
            out.push(a[1] + (b[1] - a[1]) * f);
            p += u64::from(self.in_rate);
        }
        let last = at(n);
        self.prev = last;
        self.pos = p - n as u64 * den;
    }
}
