use bincode::{
    config::standard, de::read::Reader, enc::write::Writer, error::DecodeError, Decode, Encode,
};
use embedded_io::{Read, ReadReady, Write, WriteReady};

use super::{application_layer::ApplicationPacket, hc12::HC12};

#[derive(Debug, Clone, Copy, Encode, Decode)]
#[allow(non_camel_case_types)]
pub enum LinkLayerPayload {
    Payload(ApplicationPacket),
    ACK,
    NACK,
}

#[derive(Debug, Clone, Copy, Encode, Decode, Hash)]
pub enum Device {
    Ejector,
    Deployer,
    Icarus,
    Atmega,
    Pi,
    Tester,
}

#[derive(Debug, Clone, Copy, Encode, Decode)]
pub struct LinkPacket {
    pub from_device: Device,
    pub to_device: Device,
    pub route_through: Option<Device>,
    pub payload: LinkLayerPayload,
    checksum: u32,
}

impl LinkPacket {
    // FNV1A hash
    pub fn checksum(&self) -> u32 {
        let mut hash: u32 = 0x811c9dc5;
        let fnv_prime: u32 = 16777619;

        hash ^= self.from_device as u32;
        hash = hash.wrapping_mul(fnv_prime);
        hash ^= self.to_device as u32;
        hash = hash.wrapping_mul(fnv_prime);
        if let Some(route) = self.route_through {
            hash ^= route as u32;
            hash = hash.wrapping_mul(fnv_prime);
        }

        for byte in bincode::encode_to_vec(&self.payload, standard()).unwrap() {
            hash ^= byte as u32;
            hash = hash.wrapping_mul(fnv_prime);
        }

        hash
    }

    // Set the checksum field to the correct value
    pub fn set_checksum(&mut self) {
        self.checksum = self.checksum();
    }

    // Verify the checksum field is correct
    pub fn verify_checksum(&self) -> bool {
        self.checksum == self.checksum()
    }
}

// Device to act as a link layer connection, from a embedded_hal::Read/Write/ReadReady/WriteReady
// to a bincode::Reader/Writer
pub struct LinkLayerDevice<D> {
    pub device: D,
}

impl<D, P> Reader for LinkLayerDevice<HC12<D, P>>
where
    HC12<D, P>: ReadReady + Read,
{
    fn read(&mut self, bytes: &mut [u8]) -> Result<(), DecodeError> {
        let device_avialable = self.device.bytes_available();
        let wanted_bytes = bytes.len();

        match device_avialable < wanted_bytes {
            true => Err(DecodeError::UnexpectedEnd {
                additional: wanted_bytes - device_avialable,
            }),

            false => match self.device.read(bytes) {
                Ok(_) => Ok(()),
                Err(_) => Err(DecodeError::Other("Underlying Device issue!")),
            },
        }
    }
}

impl<D, P> Writer for LinkLayerDevice<HC12<D, P>>
where
    HC12<D, P>: WriteReady + Write,
{
    fn write(&mut self, bytes: &[u8]) -> Result<(), bincode::error::EncodeError> {
        match self.device.max_bytes_to_write() < bytes.len() {
            true => Err(bincode::error::EncodeError::UnexpectedEnd),

            false => match self.write(bytes) {
                Ok(_) => Ok(()),
                Err(_) => Err(bincode::error::EncodeError::Other(
                    "Underlying Device issue!",
                )),
            },
        }
    }
}
