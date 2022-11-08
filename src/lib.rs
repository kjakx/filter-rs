use std::f64::consts::PI;
use dasp_ring_buffer;

pub trait Filter {
    type Item;

    fn process(&mut self, input: Self::Item) -> Self::Item;
}

#[derive(Copy, Clone, PartialEq)]
pub enum IIRBiquadKind {
    LPF,
    HPF,
    BPF,
    BEF,
}

pub struct IIRBiquad {
    a: [f64; 3],
    b: [f64; 3],
    input_buffer: dasp_ring_buffer::Fixed<[f64; 3]>,
    output_buffer: dasp_ring_buffer::Fixed<[f64; 3]>,
}

impl IIRBiquad {
    pub fn new(kind: IIRBiquadKind, fc: f64, q: f64, sample_rate: usize) -> Self {
        let (a, b) = Self::_calc_coefs(kind, fc, q, sample_rate);
        let input_buffer = dasp_ring_buffer::Fixed::from([0.0; 3]);
        let output_buffer = dasp_ring_buffer::Fixed::from([0.0; 3]);
        IIRBiquad {
            a,
            b,
            input_buffer,
            output_buffer,
        }
    }

    pub fn update(&mut self, kind: IIRBiquadKind, fc: f64, q: f64, sample_rate: usize) {
        (self.a, self.b) = Self::_calc_coefs(kind, fc, q, sample_rate);
    }

    pub fn reset_buffer(&mut self) {
        self.input_buffer = dasp_ring_buffer::Fixed::from([0.0; 3]);
        self.output_buffer = dasp_ring_buffer::Fixed::from([0.0; 3]);
    }

    fn _calc_coefs(kind: IIRBiquadKind, fc: f64, q: f64, sample_rate: usize) -> ([f64; 3], [f64; 3]) {
        let fc = 0.5 * (fc * PI / sample_rate as f64).tan() / PI;
        let mut a = [0.0; 3];
        let mut b = [0.0; 3];
        match kind {
            IIRBiquadKind::LPF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[1] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[2] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = 4.0 * PI * PI * fc * fc / denom;
                b[1] = 8.0 * PI * PI * fc * fc / denom;
                b[2] = 4.0 * PI * PI * fc * fc / denom;
            },
            IIRBiquadKind::HPF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[1] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[2] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = 1.0 / denom;
                b[1] = -2.0 / denom;
                b[2] = 1.0 / denom;
            },
            IIRBiquadKind::BPF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[1] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[2] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = (2.0 * PI * fc / q) / denom;
                // b[1] = 0.0;
                b[2] = (-2.0 * PI * fc / q) / denom;
            },
            IIRBiquadKind::BEF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[1] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[2] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = (4.0 * PI * PI * fc * fc + 1.0) / denom;
                b[1] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                b[2] = (4.0 * PI * PI * fc * fc + 1.0) / denom; 
            }
        }
        (a, b)
    }
}

impl Filter for IIRBiquad {
    type Item = f64;
    
    fn process(&mut self, input: Self::Item) -> Self::Item {
        self.input_buffer.push(input);
        let mut output = self.b[0] * self.input_buffer[2]
                       + self.b[1] * self.input_buffer[1]
                       + self.b[2] * self.input_buffer[0];
        output += - self.a[1] * self.output_buffer[2]
                  - self.a[2] * self.output_buffer[1];
        self.output_buffer.push(output);
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wav_sine_500hz_3500hz() {
        use hound;
        let mut reader = hound::WavReader::open("wav/sine_500hz_3500hz.wav").unwrap();
        let spec = reader.spec();
        let samples: Vec<f64> = reader.samples().map(|x: Result<i16, hound::Error>| {
            x.unwrap() as f64 / i16::MAX as f64
        }).collect();
        let mut iir_biquad = IIRBiquad::new(IIRBiquadKind::LPF, 1000.0, 1.0 / 2.0_f64.sqrt(), spec.sample_rate as usize);
        let lowpassed: Vec<f64> = samples.iter().map(|s| iir_biquad.process(*s)).collect();
        let mut writer = hound::WavWriter::create("wav/lowpass.wav", spec).unwrap();
        for s in lowpassed.iter() {
            writer.write_sample((i16::MAX as f64 * *s) as i16).unwrap();
        }
        iir_biquad.update(IIRBiquadKind::HPF, 1000.0, 1.0 / 2.0_f64.sqrt(), spec.sample_rate as usize);
        let highpassed: Vec<f64> = samples.iter().map(|s| iir_biquad.process(*s)).collect();
        let mut writer = hound::WavWriter::create("wav/highpass.wav", spec).unwrap();
        for s in highpassed.iter() {
            writer.write_sample((i16::MAX as f64 * *s) as i16).unwrap();
        }

    }
}
