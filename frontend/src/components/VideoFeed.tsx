import { ROS_CONFIG } from '../ros/config'
import { RwtPanel } from './RwtPanel'

export default function VideoFeed() {
  return (
    <RwtPanel
      title="Camera Viewer"
      src={ROS_CONFIG.rwt.image}
      description="Provide VITE_RWT_IMAGE_URL (or a base path via VITE_RWT_BASE) pointing to visualization_rwt's image view widget."
      heightClass="h-80"
    />
  )
}
