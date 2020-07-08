pub trait MessageId {
    const ID: u8;
}

pub trait Serialize {
    type Error;

    fn ser(&self, buf: &mut [u8]) -> Result<(), Self::Error>;
    fn size_hint(&self) -> usize;
}

pub trait Deserialize {
    type Output;
    type Error;

    fn des(buf: &[u8]) -> Result<Self::Output, Self::Error>;
}