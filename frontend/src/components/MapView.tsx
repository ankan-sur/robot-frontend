import { ROS_CONFIG } from '../ros/config'
import { RwtPanel } from './RwtPanel'

export function MapView() {
  return (
    <RwtPanel
      title="Navigation Map"
      src={ROS_CONFIG.rwt.map}
      description="Set VITE_RWT_MAP_URL (or VITE_RWT_BASE) so the dashboard can embed the visualization_rwt map widget."
    />
  )
}
