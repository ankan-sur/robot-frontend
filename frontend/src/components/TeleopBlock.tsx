import { useRef, useState, useEffect } from 'react';
import { topics } from '../ros/ros';

export function TeleopBlock() {
  const [lin, setLin] = useState(0.3);
  const [ang, setAng] = useState(1.0);
  const [interfaceType, setInterfaceType] = useState<'buttons' | 'joystick'>('buttons');
  const held = useRef({ vx: 0, wz: 0 });
  const intervalRef = useRef<number | null>(null);
  const joystickRef = useRef<HTMLDivElement>(null);
  const isDraggingRef = useRef(false);
  const startPosRef = useRef({ x: 0, y: 0 });

  const MAX_LINEAR = 3.5;
  const MAX_ANGULAR = 3.5;

  const publishTwist = () => topics.cmdVel.publish({
    linear: { x: held.current.vx, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: held.current.wz },
  } as any);

  const startLoop = () => {
    if (intervalRef.current != null) return;
    intervalRef.current = window.setInterval(() => {
      publishTwist();
    }, 50);
  };

  const stopLoopIfIdle = () => {
    if (held.current.vx === 0 && held.current.wz === 0 && intervalRef.current != null) {
      window.clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
  };

  // Joystick handlers
  const handleJoystickStart = (e: MouseEvent | TouchEvent) => {
    const pos = 'touches' in e ? e.touches[0] : e;
    const rect = joystickRef.current?.getBoundingClientRect();
    if (!rect) return;
    
    startPosRef.current = { x: pos.clientX - rect.left, y: pos.clientY - rect.top };
    isDraggingRef.current = true;
    startLoop();
  };

  const handleJoystickMove = (e: MouseEvent | TouchEvent) => {
    if (!isDraggingRef.current || !joystickRef.current) return;
    
    const pos = 'touches' in e ? e.touches[0] : e;
    const rect = joystickRef.current.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;
    
    const deltaX = (pos.clientX - rect.left - startPosRef.current.x) / centerX;
    const deltaY = -(pos.clientY - rect.top - startPosRef.current.y) / centerY;
    
    held.current.vx = deltaY * lin * MAX_LINEAR;
    held.current.wz = -deltaX * ang * MAX_ANGULAR;
    
    // Clamp values
    held.current.vx = Math.max(-MAX_LINEAR, Math.min(MAX_LINEAR, held.current.vx));
    held.current.wz = Math.max(-MAX_ANGULAR, Math.min(MAX_ANGULAR, held.current.wz));
  };

  const handleJoystickEnd = () => {
    isDraggingRef.current = false;
    held.current = { vx: 0, wz: 0 };
    publishTwist();
    stopLoopIfIdle();
  };

  useEffect(() => {
    if (interfaceType === 'joystick' && joystickRef.current) {
      const el = joystickRef.current;
      
      el.addEventListener('mousedown', handleJoystickStart);
      el.addEventListener('touchstart', handleJoystickStart);
      window.addEventListener('mousemove', handleJoystickMove);
      window.addEventListener('touchmove', handleJoystickMove);
      window.addEventListener('mouseup', handleJoystickEnd);
      window.addEventListener('touchend', handleJoystickEnd);
      
      return () => {
        el.removeEventListener('mousedown', handleJoystickStart);
        el.removeEventListener('touchstart', handleJoystickStart);
        window.removeEventListener('mousemove', handleJoystickMove);
        window.removeEventListener('touchmove', handleJoystickMove);
        window.removeEventListener('mouseup', handleJoystickEnd);
        window.removeEventListener('touchend', handleJoystickEnd);
      };
    }
  }, [interfaceType]);

  return (
    <div className="space-y-3">
      <div className="flex justify-between items-center mb-4">
        <div className="text-base font-medium text-blue-800">Teleop Controls</div>
        <select 
          value={interfaceType} 
          onChange={(e) => setInterfaceType(e.target.value as 'buttons' | 'joystick')}
          className="px-3 py-1 rounded border border-blue-300 text-sm"
        >
          <option value="buttons">Button Mode</option>
          <option value="joystick">Joystick Mode</option>
        </select>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 items-center">
        <div className="flex items-center gap-3">
          <label className="text-sm text-slate-600 w-20">Linear</label>
          <input
            className="flex-1"
            type="range"
            min="0"
            max={MAX_LINEAR}
            step="0.1"
            value={lin}
            onChange={e => {
              const v = +e.target.value;
              setLin(v);
              if (held.current.vx !== 0) {
                held.current.vx = Math.sign(held.current.vx) * v;
              }
            }}
          />
          <div className="w-14 text-right font-mono text-sm text-slate-700">{lin.toFixed(1)}</div>
        </div>
        <div className="flex items-center gap-3">
          <label className="text-sm text-slate-600 w-20">Angular</label>
          <input
            className="flex-1"
            type="range"
            min="0"
            max={MAX_ANGULAR}
            step="0.1"
            value={ang}
            onChange={e => {
              const v = +e.target.value;
              setAng(v);
              if (held.current.wz !== 0) {
                held.current.wz = Math.sign(held.current.wz) * v;
              }
            }}
          />
          <div className="w-14 text-right font-mono text-sm text-slate-700">{ang.toFixed(1)}</div>
        </div>
      </div>

      {interfaceType === 'buttons' ? (
        <div className="flex flex-wrap gap-2">
          <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
            onMouseDown={() => { held.current.vx = lin; startLoop(); }}
            onMouseUp={() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle(); }}
            onTouchStart={(e) => { e.preventDefault(); held.current.vx = lin; startLoop(); }}
            onTouchEnd={(e) => { e.preventDefault(); held.current.vx = 0; publishTwist(); stopLoopIfIdle(); }}
          >Forward</button>
          <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
            onMouseDown={() => { held.current.vx = -lin; startLoop(); }}
            onMouseUp={() => { held.current.vx = 0; publishTwist(); stopLoopIfIdle(); }}
            onTouchStart={(e) => { e.preventDefault(); held.current.vx = -lin; startLoop(); }}
            onTouchEnd={(e) => { e.preventDefault(); held.current.vx = 0; publishTwist(); stopLoopIfIdle(); }}
          >Back</button>
          <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
            onMouseDown={() => { held.current.wz = ang; startLoop(); }}
            onMouseUp={() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle(); }}
            onTouchStart={(e) => { e.preventDefault(); held.current.wz = ang; startLoop(); }}
            onTouchEnd={(e) => { e.preventDefault(); held.current.wz = 0; publishTwist(); stopLoopIfIdle(); }}
          >Left</button>
          <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
            onMouseDown={() => { held.current.wz = -ang; startLoop(); }}
            onMouseUp={() => { held.current.wz = 0; publishTwist(); stopLoopIfIdle(); }}
            onTouchStart={(e) => { e.preventDefault(); held.current.wz = -ang; startLoop(); }}
            onTouchEnd={(e) => { e.preventDefault(); held.current.wz = 0; publishTwist(); stopLoopIfIdle(); }}
          >Right</button>
          <button className="px-4 py-2 border rounded bg-slate-100 hover:bg-slate-200"
            onClick={() => { held.current = { vx: 0, wz: 0 }; publishTwist(); stopLoopIfIdle(); }}
          >Stop</button>
        </div>
      ) : (
        <div 
          ref={joystickRef}
          className="w-48 h-48 mx-auto bg-slate-100 rounded-full border-4 border-blue-300 relative cursor-pointer touch-none"
        >
          <div className="absolute inset-0 flex items-center justify-center text-blue-400 pointer-events-none">
            Drag here
          </div>
          {isDraggingRef.current && (
            <div className="absolute inset-0 flex items-center justify-center">
              <div className="text-xs text-blue-600">
                Lin: {held.current.vx.toFixed(1)} | Ang: {held.current.wz.toFixed(1)}
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}