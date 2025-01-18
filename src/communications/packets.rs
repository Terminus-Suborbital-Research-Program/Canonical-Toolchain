use alloc::vec::Vec;
use bincode::{Decode, Encode};

#[derive(Debug, Clone, Copy, Encode, Decode, PartialEq)]
pub struct ScientificPacket {
    pub packets: u32,
    pub temperature: f32,
}

#[derive(Debug, Clone, Copy, Encode, Decode, PartialEq, Eq)]
pub enum CommandPacket {
    SyncTime(u32),
    MoveServoDegrees(i32),
}

#[derive(Debug, Clone, Copy, Encode, Decode, PartialEq, Eq)]
pub enum PacketType {
    Scientific,
    Command,
}

impl PacketType {
    fn from_variation(variation: &PacketVariation) -> Self {
        match variation {
            PacketVariation::Scientific(_) => Self::Scientific,
            PacketVariation::Command(_) => Self::Command,
        }
    }
}

#[derive(Debug, Clone, Copy, Encode, Decode)]
pub enum PacketVariation {
    Scientific(ScientificPacket),
    Command(CommandPacket),
}

#[derive(Debug, Clone, Encode, Decode)]
pub struct PacketFrame {
    packet_type: PacketType,
    data: Vec<u8>,
}

impl From<PacketVariation> for PacketFrame {
    fn from(value: PacketVariation) -> Self {
        let packet_type = PacketType::from_variation(&value);
        // TODO: How can this fail?
        let data: Vec<u8> =
            bincode::encode_to_vec(value, bincode::config::standard()).unwrap_or(Vec::new());

        Self { packet_type, data }
    }
}
