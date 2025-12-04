/**
 * Demo Mode Entry Point
 * 
 * This file provides the demo mode entrypoint and routing logic.
 * The demo UI can be accessed via:
 *   - /?demo=true
 *   - /demo
 */

import React from 'react';
import AppDemo from './AppDemo';

/**
 * Check if we should render the demo UI
 */
export function isDemoMode(): boolean {
  // Check URL query parameter
  const params = new URLSearchParams(window.location.search);
  if (params.get('demo') === 'true') {
    return true;
  }
  
  // Check pathname
  if (window.location.pathname === '/demo' || window.location.pathname === '/demo/') {
    return true;
  }
  
  // Check environment variable (for build-time configuration)
  if (import.meta.env.VITE_DEMO_MODE === 'true') {
    return true;
  }
  
  return false;
}

/**
 * Demo App wrapper for routing
 */
export const DemoApp: React.FC = () => {
  return <AppDemo />;
};

export default DemoApp;
