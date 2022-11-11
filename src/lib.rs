use std::f32::consts::PI;
use dasp_ring_buffer;
use dasp_signal::{self as signal, Signal};
use dasp_sample::Sample;
use dasp_frame::Frame;

#[derive(Copy, Clone, PartialEq)]
pub enum IIRBiquadKind {
    LPF,
    HPF,
    BPF,
    BEF,
}

#[derive(Copy, Clone, PartialEq)]
pub enum IIRBiquadChannel {
    Mono = 1,
    Stereo = 2,
}

#[derive(Clone)]
pub struct IIRBiquadProcessor {
    input_buffer : dasp_ring_buffer::Fixed<[f32; 3]>,
    output_buffer: dasp_ring_buffer::Fixed<[f32; 2]>,
}

impl IIRBiquadProcessor {
    pub fn new() -> Self {
        IIRBiquadProcessor {
            input_buffer : dasp_ring_buffer::Fixed::from([0.0; 3]),
            output_buffer: dasp_ring_buffer::Fixed::from([0.0; 2]),
        }
    }

    pub fn process(&mut self, input: f32, coefs: &IIRBiquadCoefficients) -> f32 {
        self.input_buffer.push(input);
        let output = coefs.b[0] * self.input_buffer[2]
                   + coefs.b[1] * self.input_buffer[1]
                   + coefs.b[2] * self.input_buffer[0]
                   - coefs.a[0] * self.output_buffer[1]
                   - coefs.a[1] * self.output_buffer[0];
        self.output_buffer.push(output);
        output
    }

    pub fn reset(&mut self) {
        self.input_buffer = dasp_ring_buffer::Fixed::from([0.0; 3]);
        self.output_buffer = dasp_ring_buffer::Fixed::from([0.0; 2]);
    }
}

pub struct IIRBiquadCoefficients {
    a: [f32; 2],
    b: [f32; 3],
}

impl IIRBiquadCoefficients {
    pub fn new(kind: IIRBiquadKind, fc: f32, q: f32, sample_rate: f32) -> Self {
        let fc = 0.5 * (fc * PI / sample_rate).tan() / PI;
        let mut a = [0.0; 2];
        let mut b = [0.0; 3];
        match kind {
            IIRBiquadKind::LPF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[0] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[1] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = 4.0 * PI * PI * fc * fc / denom;
                b[1] = 8.0 * PI * PI * fc * fc / denom;
                b[2] = 4.0 * PI * PI * fc * fc / denom;
            },
            IIRBiquadKind::HPF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[0] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[1] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = 1.0 / denom;
                b[1] = -2.0 / denom;
                b[2] = 1.0 / denom;
            },
            IIRBiquadKind::BPF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[0] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[1] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = (2.0 * PI * fc / q) / denom;
                // b[1] = 0.0;
                b[2] = (-2.0 * PI * fc / q) / denom;
            },
            IIRBiquadKind::BEF => {
                let denom = 1.0 + 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc;
                a[0] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                a[1] = (1.0 - 2.0 * PI * fc / q + 4.0 * PI * PI * fc * fc) / denom;
                b[0] = (4.0 * PI * PI * fc * fc + 1.0) / denom;
                b[1] = (8.0 * PI * PI * fc * fc - 2.0) / denom;
                b[2] = (4.0 * PI * PI * fc * fc + 1.0) / denom; 
            }
        }
        IIRBiquadCoefficients{ a, b }
    }
}

pub struct IIRBiquad {
    kind: IIRBiquadKind,
    fc: f32,
    q: f32,
    coefs: IIRBiquadCoefficients,
    processors: Vec<IIRBiquadProcessor>,
    sample_rate: f32,
}

impl IIRBiquad {
    pub fn new(kind: IIRBiquadKind, fc: f32, q: f32, channel: IIRBiquadChannel, sample_rate: f32) -> Self {
        let coefs = IIRBiquadCoefficients::new(kind, fc, q, sample_rate);
        let processors = vec![IIRBiquadProcessor::new(); channel as usize];
        IIRBiquad {
            kind,
            fc,
            q,
            coefs,
            processors,
            sample_rate,
        }
    }

    pub fn update(&mut self, fc: f32, q: f32) {
        self.coefs = IIRBiquadCoefficients::new(self.kind, fc, q, self.sample_rate);
    }

    pub fn change_kind(&mut self, kind: IIRBiquadKind) {
        self.kind = kind;
        self.update(self.fc, self.q);
        self.processors.iter_mut().for_each(|p| p.reset());
    }

    pub fn change_sample_rate(&mut self, sample_rate: f32) {
        self.sample_rate = sample_rate;
        self.update(self.fc, self.q);
    }

    // TODO: use dasp_frame
    pub fn process(&mut self, input: Vec<f32>) -> Vec<f32> {
        self.processors.iter_mut()
        .zip(input.iter())
        .map(|(p, i)| {
            p.process(*i, &self.coefs)
        })
        .collect::<Vec<f32>>()
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
        let samples: Vec<f32> = reader
            .samples::<i16>()
            .filter_map(|x| x.ok())
            .map(Sample::to_sample::<f32>)
            .collect();

        let mut iir_biquad = IIRBiquad::new(
            IIRBiquadKind::LPF,
            1000.0,
            1.0 / 2.0_f32.sqrt(), 
            IIRBiquadChannel::Mono,
            spec.sample_rate as f32
        );

        // apply low-pass filter
        let lowpassed: Vec<Vec<f32>> = samples.iter().map(|s| iir_biquad.process(vec![*s])).collect();
        let mut writer = hound::WavWriter::create("wav/lowpass.wav", spec).unwrap();
        for s in lowpassed.iter() {
            writer.write_sample((i16::MAX as f32 * s[0]) as i16).unwrap();
        }
        
        // apply high-pass filter
        iir_biquad.change_kind(IIRBiquadKind::HPF);
        let highpassed: Vec<Vec<f32>> = samples.iter().map(|s| iir_biquad.process(vec![*s])).collect();
        let mut writer = hound::WavWriter::create("wav/highpass.wav", spec).unwrap();
        for s in highpassed.iter() {
            writer.write_sample((i16::MAX as f32 * s[0]) as i16).unwrap();
        }

    }
}
