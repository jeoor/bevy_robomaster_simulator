use crate::layout::*;
use crate::shm::{ShmError, ShmRegion};
use crate::triple_buffer::TripleBufferConsumer;

pub struct ShmSubscriber {
    meta_region: ShmRegion,
    overlay_region: Option<ShmRegion>,
}

impl ShmSubscriber {
    pub fn connect() -> Result<Self, ShmError> {
        let meta_region = ShmRegion::open(SHM_NAME_META, size_of::<ShmMetaRegion>())?;

        unsafe {
            let meta = meta_region.as_ref::<ShmMetaRegion>();
            if meta.header.magic != SHM_MAGIC {
                return Err(ShmError::InvalidSize);
            }
            if meta.header.version != SHM_VERSION {
                return Err(ShmError::InvalidSize);
            }
        }

        let overlay_region = ShmRegion::open(SHM_NAME_OVERLAY, size_of::<OverlayTripleBuffer>()).ok();
        Ok(Self { meta_region, overlay_region })
    }

    pub fn recv_overlay(&mut self) -> Option<AimOverlay> {
        let region = self.overlay_region.as_mut()?;
        unsafe {
            let channel = region.as_mut::<OverlayTripleBuffer>();
            TripleBufferConsumer::new(&channel.state, &mut channel.read_idx, &channel.slots)
                .borrow().copied()
        }
    }

    pub fn recv_gimbal_cmd(&mut self) -> Option<GimbalCmd> {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let mut consumer = TripleBufferConsumer::new(
                &meta.gimbal_cmd.state,
                &mut meta.gimbal_cmd.read_idx,
                &meta.gimbal_cmd.slots,
            );

            consumer.borrow().copied()
        }
    }

    pub fn has_gimbal_cmd(&self) -> bool {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            (meta
                .gimbal_cmd
                .state
                .load(std::sync::atomic::Ordering::Acquire)
                & FLAG_NEW)
                != 0
        }
    }

    pub fn chassis_observation(&self) -> Option<ChassisObservation> {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            let observation = meta.chassis_observation;
            if observation.timestamp_ns == 0 {
                None
            } else {
                Some(observation)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triple_buffer::TripleBufferProducer;

    #[test]
    fn display_channel_delivers_latest_packet_without_consuming_commands() {
        let name = format!("sp_overlay_test_{}", std::process::id());
        let mut producer = ShmRegion::create(&name, size_of::<OverlayTripleBuffer>()).unwrap();
        unsafe {
            *producer.as_mut::<OverlayTripleBuffer>() = OverlayTripleBuffer::default();
        }
        let mut subscriber = ShmSubscriber {
            meta_region: ShmRegion::create(&format!("{name}_meta"), size_of::<ShmMetaRegion>()).unwrap(),
            overlay_region: Some(ShmRegion::open(&name, size_of::<OverlayTripleBuffer>()).unwrap()),
        };
        for stamp in 1..=3 {
            unsafe {
                let channel = producer.as_mut::<OverlayTripleBuffer>();
                let mut writer = TripleBufferProducer::new(
                    &channel.state, &mut channel.write_idx, &mut channel.slots,
                );
                *writer.borrow_mut() = AimOverlay { timestamp_ns: stamp, armor_count: 2, ..Default::default() };
                writer.publish();
            }
        }
        let packet = subscriber.recv_overlay().unwrap();
        assert_eq!((packet.timestamp_ns, packet.armor_count), (3, 2));
        assert!(subscriber.recv_overlay().is_none());
        assert!(!subscriber.has_gimbal_cmd());
    }
}
