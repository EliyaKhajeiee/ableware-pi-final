import { useState, useEffect, useRef, useCallback } from 'react';
import { ChevronUp, ChevronDown, Power, AlertTriangle, Activity, Zap } from 'lucide-react';
import { toast } from 'sonner';

// ---- Protocol types ------------------------------------------------

interface SimulationState {
  position: number;
  velocity: number;
  acceleration: number;
  pwm: number;
  emergency_stopped: boolean;
  stalled: boolean;
  target_position: number;
  at_target: boolean;
  is_stable: boolean;
}

interface HistoryEntry {
  command: string;
  source: string;
  timestamp: number;
}

interface StateUpdate {
  type: 'state_update';
  last_command: string | null;
  last_command_source: string | null;
  simulation_state: SimulationState;
  command_history: HistoryEntry[];
  pi_connected: boolean;
  sim_connected: boolean;
  arduino_connected: boolean;
  position_steps: number;
}

// ---- Constants -----------------------------------------------------

const ACT_STROKE = 0.1016;
const HUB_WS_URL =
  window.location.hostname === 'localhost'
    ? 'ws://localhost:8000/ws/dashboard'
    : `ws://${window.location.hostname}:8000/ws/dashboard`;

// ---- Main component ------------------------------------------------

export function MainControl() {
  const [hubConnected, setHubConnected] = useState(false);
  const [piConnected, setPiConnected] = useState(false);
  const [simConnected, setSimConnected] = useState(false);
  const [arduinoConnected, setArduinoConnected] = useState(false);
  const [positionSteps, setPositionSteps] = useState(0);
  const [simState, setSimState] = useState<SimulationState | null>(null);
  const [lastCommand, setLastCommand] = useState<string | null>(null);
  const [lastSource, setLastSource] = useState<string | null>(null);
  const [history, setHistory] = useState<HistoryEntry[]>([]);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  const connect = useCallback(() => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return;
    const ws = new WebSocket(HUB_WS_URL);
    wsRef.current = ws;
    ws.onopen = () => { setHubConnected(true); toast.success('Hub connected'); };
    ws.onmessage = (event) => {
      try {
        const data: StateUpdate = JSON.parse(event.data);
        if (data.type === 'state_update') {
          setSimState(data.simulation_state);
          setPiConnected(data.pi_connected);
          setSimConnected(data.sim_connected ?? false);
          setArduinoConnected(data.arduino_connected ?? false);
          setPositionSteps(data.position_steps ?? 0);
          setLastCommand(data.last_command);
          setLastSource(data.last_command_source);
          setHistory(data.command_history.slice(0, 6));
        }
      } catch { /* ignore */ }
    };
    ws.onerror = () => toast.error('Connection error');
    ws.onclose = () => {
      setHubConnected(false);
      setPiConnected(false);
      wsRef.current = null;
      reconnectTimer.current = setTimeout(connect, 3000);
    };
  }, []);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current);
      wsRef.current?.close();
    };
  }, [connect]);

  const sendCommand = (cmd: string) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      toast.error('Not connected');
      return;
    }
    wsRef.current.send(JSON.stringify({ type: 'command', command: cmd, timestamp: Date.now() / 1000, source: 'manual' }));
  };

  const positionPct = simState ? Math.round((simState.position / ACT_STROKE) * 100) : 0;
  const stepPct = Math.round((positionSteps / 7) * 100);
  const isEmergency = simState?.emergency_stopped ?? false;
  const isMoving = simState && !simState.at_target;

  return (
    <div className="min-h-screen bg-[#05050a] flex flex-col" style={{ fontFamily: "'SF Mono', 'JetBrains Mono', 'Fira Code', monospace" }}>

      {/* ── Top bar ──────────────────────────────────────────── */}
      <div className="flex items-center justify-between px-10 py-5 border-b border-white/[0.06]">
        <div className="flex items-center gap-4">
          <div className="w-2 h-8 bg-sky-400 rounded-sm" style={{ boxShadow: '0 0 12px #38bdf8, 0 0 24px #38bdf840' }} />
          <div>
            <div className="text-2xl font-bold tracking-[0.15em] text-white uppercase">Ableware</div>
            <div className="text-[10px] tracking-[0.3em] text-slate-500 uppercase mt-0.5">Assistive Lift Control System</div>
          </div>
        </div>

        {/* Connection nodes */}
        <div className="flex items-center gap-8">
          {[
            { label: 'HUB', active: hubConnected, color: '#4ade80', glow: '#4ade8060' },
            { label: 'PI', active: piConnected, color: '#a78bfa', glow: '#a78bfa60' },
            { label: 'SIM', active: simConnected, color: '#38bdf8', glow: '#38bdf860' },
            { label: 'ARDUINO', active: arduinoConnected, color: '#22d3ee', glow: '#22d3ee60' },
          ].map(({ label, active, color, glow }) => (
            <div key={label} className="flex flex-col items-center gap-2">
              <div
                className="w-2.5 h-2.5 rounded-full transition-all duration-700"
                style={{
                  backgroundColor: active ? color : '#1e293b',
                  boxShadow: active ? `0 0 8px ${color}, 0 0 16px ${glow}` : 'none',
                }}
              />
              <span className="text-[9px] tracking-[0.25em] uppercase" style={{ color: active ? color : '#334155' }}>
                {label}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* ── Main ─────────────────────────────────────────────── */}
      <div className="flex-1 grid grid-cols-5 min-h-0">

        {/* ── Left: Telemetry (2/5) ───────────────────────────── */}
        <div className="col-span-2 flex flex-col gap-0 border-r border-white/[0.06] p-10">

          {/* System status pill */}
          <div className="mb-8">
            <div
              className={`inline-flex items-center gap-2 px-4 py-1.5 rounded-full text-xs tracking-[0.2em] uppercase border transition-all duration-500 ${
                isEmergency
                  ? 'border-red-500/50 bg-red-500/10 text-red-400'
                  : isMoving
                  ? 'border-amber-500/50 bg-amber-500/10 text-amber-400'
                  : 'border-emerald-500/30 bg-emerald-500/[0.08] text-emerald-400'
              }`}
              style={isEmergency ? { boxShadow: '0 0 20px #ef444420' } : isMoving ? { boxShadow: '0 0 20px #f59e0b20' } : {}}
            >
              <div className={`w-1.5 h-1.5 rounded-full ${isMoving && !isEmergency ? 'animate-pulse' : ''}`}
                style={{ backgroundColor: isEmergency ? '#f87171' : isMoving ? '#fbbf24' : '#4ade80' }} />
              {isEmergency ? 'Emergency Stop Active' : isMoving ? 'Actuator Moving' : 'System Stable'}
            </div>
          </div>

          {/* Big position number */}
          <div className="mb-2">
            <div className="text-xs tracking-[0.25em] uppercase text-slate-300 font-semibold mb-3">Lift Position</div>
            <div className="flex items-end gap-3 mb-5">
              <span
                className="text-8xl font-bold text-white leading-none"
                style={{ textShadow: '0 0 40px #38bdf840' }}
              >
                {positionPct}
              </span>
              <span className="text-3xl text-slate-400 mb-2">%</span>
              <span className="text-slate-500 text-base mb-2 ml-1">
                {simState ? (simState.position * 1000).toFixed(1) : '0.0'} mm
              </span>
            </div>

            {/* Position bar */}
            <div className="relative h-2 bg-white/[0.04] rounded-full overflow-hidden">
              <div
                className="absolute inset-y-0 left-0 rounded-full transition-all duration-700"
                style={{
                  width: `${positionPct}%`,
                  background: 'linear-gradient(90deg, #0ea5e9, #38bdf8)',
                  boxShadow: positionPct > 0 ? '0 0 12px #38bdf8' : 'none',
                }}
              />
            </div>
          </div>

          {/* Step indicator */}
          <div className="mb-8">
            <div className="flex items-center justify-between mb-3">
              <span className="text-xs tracking-[0.25em] uppercase text-slate-300 font-semibold">Step Position</span>
              <span className="text-sm text-white font-bold">{positionSteps}<span className="text-slate-600 font-normal"> / 7</span></span>
            </div>
            <div className="flex gap-2">
              {Array.from({ length: 8 }, (_, i) => (
                <div
                  key={i}
                  className="flex-1 h-1.5 rounded-full transition-all duration-300"
                  style={{
                    backgroundColor: i < positionSteps ? '#a78bfa' : i === positionSteps ? '#7c3aed' : '#1e1e2e',
                    boxShadow: i < positionSteps ? '0 0 6px #a78bfa80' : 'none',
                  }}
                />
              ))}
            </div>
          </div>

          {/* Metrics grid */}
          <div className="flex-1 space-y-0">
            <div className="text-[10px] tracking-[0.35em] uppercase text-slate-600 mb-4">Telemetry</div>
            {[
              { label: 'Velocity', value: simState ? (simState.velocity * 1000).toFixed(1) : '—', unit: 'mm/s' },
              { label: 'PWM Output', value: simState ? simState.pwm.toFixed(3) : '—', unit: '' },
              { label: 'Target', value: simState ? (simState.target_position * 1000).toFixed(1) : '—', unit: 'mm' },
              { label: 'Stall', value: simState?.stalled ? 'DETECTED' : 'Clear', unit: '' },
              { label: 'Last Cmd', value: lastCommand || '—', unit: lastCommand ? `via ${lastSource}` : '' },
            ].map(({ label, value, unit }) => (
              <div key={label} className="flex items-center justify-between py-3 border-b border-white/[0.04]">
                <span className="text-xs tracking-[0.15em] uppercase text-slate-300 font-semibold">{label}</span>
                <span className="text-base text-white font-bold">
                  {value}
                  {unit && <span className="text-slate-500 text-sm font-normal ml-1.5">{unit}</span>}
                </span>
              </div>
            ))}
          </div>
        </div>

        {/* ── Right: Controls (3/5) ──────────────────────────── */}
        <div className="col-span-3 flex flex-col p-10 gap-5">

          <div className="text-[10px] tracking-[0.35em] uppercase text-slate-600 mb-2">Manual Control</div>

          {/* Resume */}
          <button
            onClick={() => sendCommand('START')}
            disabled={!hubConnected}
            className="w-full py-5 rounded-xl border text-sm tracking-[0.2em] uppercase font-semibold transition-all duration-200 disabled:cursor-not-allowed"
            style={hubConnected ? {
              borderColor: '#166534',
              color: '#4ade80',
              backgroundColor: 'transparent',
            } : {
              borderColor: '#1e293b',
              color: '#1e293b',
              backgroundColor: 'transparent',
            }}
            onMouseEnter={e => { if (hubConnected) (e.currentTarget as HTMLButtonElement).style.backgroundColor = '#052e16'; }}
            onMouseLeave={e => { (e.currentTarget as HTMLButtonElement).style.backgroundColor = 'transparent'; }}
          >
            <Power className="w-4 h-4 inline mr-3" />
            Resume System
          </button>

          {/* UP */}
          <button
            onClick={() => sendCommand('UP')}
            disabled={!hubConnected || isEmergency}
            className="flex-1 w-full rounded-2xl border-2 font-bold tracking-[0.15em] uppercase transition-all duration-150 active:scale-[0.98] disabled:cursor-not-allowed flex flex-col items-center justify-center gap-3"
            style={hubConnected && !isEmergency ? {
              borderColor: '#0369a1',
              color: '#38bdf8',
              backgroundColor: '#0c1a2e',
            } : {
              borderColor: '#0f172a',
              color: '#1e293b',
              backgroundColor: 'transparent',
            }}
            onMouseEnter={e => { if (hubConnected && !isEmergency) { const b = e.currentTarget as HTMLButtonElement; b.style.borderColor = '#38bdf8'; b.style.boxShadow = '0 0 30px #38bdf820, inset 0 0 30px #38bdf808'; }}}
            onMouseLeave={e => { const b = e.currentTarget as HTMLButtonElement; b.style.borderColor = hubConnected && !isEmergency ? '#0369a1' : '#0f172a'; b.style.boxShadow = 'none'; }}
          >
            <ChevronUp className="w-14 h-14" />
            <span className="text-2xl">UP</span>
          </button>

          {/* DOWN */}
          <button
            onClick={() => sendCommand('DOWN')}
            disabled={!hubConnected || isEmergency}
            className="flex-1 w-full rounded-2xl border-2 font-bold tracking-[0.15em] uppercase transition-all duration-150 active:scale-[0.98] disabled:cursor-not-allowed flex flex-col items-center justify-center gap-3"
            style={hubConnected && !isEmergency ? {
              borderColor: '#0369a1',
              color: '#38bdf8',
              backgroundColor: '#0c1a2e',
            } : {
              borderColor: '#0f172a',
              color: '#1e293b',
              backgroundColor: 'transparent',
            }}
            onMouseEnter={e => { if (hubConnected && !isEmergency) { const b = e.currentTarget as HTMLButtonElement; b.style.borderColor = '#38bdf8'; b.style.boxShadow = '0 0 30px #38bdf820, inset 0 0 30px #38bdf808'; }}}
            onMouseLeave={e => { const b = e.currentTarget as HTMLButtonElement; b.style.borderColor = hubConnected && !isEmergency ? '#0369a1' : '#0f172a'; b.style.boxShadow = 'none'; }}
          >
            <ChevronDown className="w-14 h-14" />
            <span className="text-2xl">DOWN</span>
          </button>

          {/* Emergency stop */}
          <button
            onClick={() => sendCommand('STOP')}
            disabled={!hubConnected}
            className="w-full py-7 rounded-xl border-2 font-bold tracking-[0.2em] uppercase text-lg transition-all duration-200 disabled:cursor-not-allowed flex items-center justify-center gap-4"
            style={isEmergency ? {
              borderColor: '#ef4444',
              color: '#fca5a5',
              backgroundColor: '#1f0a0a',
              boxShadow: '0 0 40px #ef444430',
            } : hubConnected ? {
              borderColor: '#7f1d1d',
              color: '#ef4444',
              backgroundColor: '#0d0505',
            } : {
              borderColor: '#1e293b',
              color: '#1e293b',
              backgroundColor: 'transparent',
            }}
            onMouseEnter={e => { if (hubConnected && !isEmergency) { const b = e.currentTarget as HTMLButtonElement; b.style.borderColor = '#ef4444'; b.style.boxShadow = '0 0 30px #ef444430'; }}}
            onMouseLeave={e => { if (!isEmergency) { const b = e.currentTarget as HTMLButtonElement; b.style.borderColor = hubConnected ? '#7f1d1d' : '#1e293b'; b.style.boxShadow = 'none'; }}}
          >
            <AlertTriangle className="w-6 h-6" />
            {isEmergency ? 'E-STOP ACTIVE — Click to Resume' : 'Emergency Stop'}
          </button>
        </div>
      </div>

      {/* ── Command log ──────────────────────────────────────── */}
      <div className="border-t border-white/[0.05] px-10 py-4">
        <div className="flex items-center gap-8 overflow-x-auto">
          <div className="flex items-center gap-2 flex-shrink-0">
            <Activity className="w-3 h-3 text-slate-600" />
            <span className="text-[9px] tracking-[0.3em] uppercase text-slate-600">Log</span>
          </div>
          {history.length === 0 && (
            <span className="text-[11px] text-slate-700 font-mono">No commands yet</span>
          )}
          {history.map((entry, i) => (
            <div key={i} className={`flex items-center gap-3 flex-shrink-0 font-mono text-xs transition-opacity ${i === 0 ? 'opacity-100' : 'opacity-40'}`}>
              <span className="text-slate-600">{new Date(entry.timestamp * 1000).toLocaleTimeString('en', { hour12: false })}</span>
              <span className={`font-bold ${entry.source === 'voice' ? 'text-violet-400' : 'text-sky-400'}`}>{entry.command}</span>
              <span className="text-slate-600">{entry.source}</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
