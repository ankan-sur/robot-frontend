/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_ROSBRIDGE_URL?: string;
  readonly VITE_VIDEO_BASE?: string;
  readonly VITE_ROSBRIDGE_FALLBACK_URL?: string;
  readonly VITE_VIDEO_FALLBACK_BASE?: string;
  readonly VITE_RWT_BASE?: string;
  readonly VITE_RWT_MAP_URL?: string;
  readonly VITE_RWT_TELEOP_URL?: string;
  readonly VITE_RWT_IMAGE_URL?: string;
  readonly VITE_ROSBOARD_URL?: string;
  readonly VITE_BACKEND_URL?: string;
  readonly VITE_API_BASE?: string;
  readonly VITE_WEBRTC_SIGNALING_URL?: string;
  readonly VITE_WEBRTC_STREAM_ID?: string;
  readonly VITE_WEBRTC_ICE_SERVERS?: string;
  // add other VITE_... variables you use here
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
