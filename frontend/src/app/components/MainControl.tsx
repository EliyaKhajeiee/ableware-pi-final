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
  const [override, setOverride] = useState(false);

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
    wsRef.current.send(JSON.stringify({ type: 'command', command: cmd, timestamp: Date.now() / 1000, source: 'manual', override }));
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
            <div className="text-4xl font-bold tracking-[0.15em] text-white uppercase">Ableware</div>
            <div className="text-xs tracking-[0.3em] text-slate-400 uppercase mt-1">Assistive Lift Control System</div>
          </div>
        </div>

        {/* Override toggle */}
        <button
          onClick={() => setOverride(o => !o)}
          className="text-[9px] font-mono tracking-widest uppercase px-2 py-1 rounded border transition-all duration-200"
          style={override ? {
            borderColor: '#ef4444', color: '#ef4444', backgroundColor: '#1f0505',
            boxShadow: '0 0 8px #ef444440'
          } : {
            borderColor: '#3f1010', color: '#7f2020', backgroundColor: 'transparent'
          }}
        >
          {override ? '⚠ OVERRIDE ON' : 'OVERRIDE'}
        </button>

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
        <div className="col-span-2 flex flex-col border-r border-white/[0.06] p-6 gap-3">

          {/* Position — hero metric */}
          <div className="rounded-2xl p-5" style={{ backgroundColor: '#0a0f1a', border: '1px solid #0e2040' }}>
            <div className="flex items-center justify-between mb-1">
              <span className="text-sm tracking-[0.2em] uppercase font-bold text-sky-400">Lift Position</span>
              <span className="text-sm font-bold text-slate-400">{simState ? (simState.position * 1000).toFixed(1) : '0.0'} mm</span>
            </div>
            <div className="flex items-end gap-2 mb-3">
              <span className="font-bold text-white leading-none" style={{ fontSize: '5rem', textShadow: '0 0 40px #38bdf860' }}>{positionPct}</span>
              <span className="text-4xl text-sky-500 font-bold mb-2">%</span>
            </div>
            <div className="h-3 bg-white/[0.05] rounded-full overflow-hidden">
              <div className="h-full rounded-full transition-all duration-700"
                style={{ width: `${positionPct}%`, background: 'linear-gradient(90deg, #0369a1, #38bdf8)', boxShadow: '0 0 10px #38bdf8' }} />
            </div>
          </div>

          {/* Step position */}
          <div className="rounded-2xl p-5" style={{ backgroundColor: '#0d0a1a', border: '1px solid #1e0e40' }}>
            <div className="flex items-center justify-between mb-3">
              <span className="text-sm tracking-[0.2em] uppercase font-bold text-violet-400">Step Position</span>
              <span className="text-3xl font-bold text-white">{positionSteps}<span className="text-violet-500 text-xl"> / 7</span></span>
            </div>
            <div className="flex gap-2">
              {Array.from({ length: 7 }, (_, i) => (
                <div key={i} className="flex-1 h-3 rounded-full transition-all duration-300"
                  style={{ backgroundColor: i < positionSteps ? '#a78bfa' : '#1e1e2e', boxShadow: i < positionSteps ? '0 0 8px #a78bfa' : 'none' }} />
              ))}
            </div>
          </div>

          {/* Velocity */}
          <div className="flex-1 rounded-2xl p-5 flex flex-col justify-between" style={{ backgroundColor: '#0a0f1a', border: '1px solid #0e2040' }}>
            <span className="text-sm tracking-[0.2em] uppercase font-bold text-sky-400">Velocity</span>
            <div className="flex items-end gap-2">
              <span className="text-5xl font-bold text-white leading-none">{simState ? (simState.velocity * 1000).toFixed(1) : '0.0'}</span>
              <span className="text-xl text-sky-600 font-bold mb-1">mm/s</span>
            </div>
            <div className="h-2 bg-white/[0.05] rounded-full overflow-hidden">
              <div className="h-full rounded-full transition-all duration-500"
                style={{ width: `${Math.min(Math.abs((simState?.velocity ?? 0) * 1000) / 50 * 100, 100)}%`, background: 'linear-gradient(90deg, #0369a1, #38bdf8)', boxShadow: '0 0 8px #38bdf8' }} />
            </div>
          </div>

          {/* PWM */}
          <div className="flex-1 rounded-2xl p-5 flex flex-col justify-between" style={{ backgroundColor: '#0d0a1a', border: '1px solid #1e0e40' }}>
            <span className="text-sm tracking-[0.2em] uppercase font-bold text-violet-400">PWM Output</span>
            <div className="flex items-end gap-2">
              <span className="text-5xl font-bold text-white leading-none">{simState ? (Math.abs(simState.pwm) * 100).toFixed(0) : '0'}</span>
              <span className="text-xl text-violet-600 font-bold mb-1">%</span>
            </div>
            <div className="h-2 bg-white/[0.05] rounded-full overflow-hidden">
              <div className="h-full rounded-full transition-all duration-500"
                style={{ width: `${Math.min(Math.abs(simState?.pwm ?? 0) * 100, 100)}%`, background: 'linear-gradient(90deg, #5b21b6, #a78bfa)', boxShadow: '0 0 8px #a78bfa' }} />
            </div>
          </div>

          {/* Target */}
          <div className="flex-1 rounded-2xl p-5 flex flex-col justify-between" style={{ backgroundColor: '#0a1410', border: '1px solid #0e3020' }}>
            <span className="text-sm tracking-[0.2em] uppercase font-bold text-emerald-400">Target Position</span>
            <div className="flex items-end gap-2">
              <span className="text-5xl font-bold text-white leading-none">{simState ? (simState.target_position * 1000).toFixed(1) : '0.0'}</span>
              <span className="text-xl text-emerald-600 font-bold mb-1">mm</span>
            </div>
            <div className="h-2 bg-white/[0.05] rounded-full overflow-hidden">
              <div className="h-full rounded-full transition-all duration-500"
                style={{ width: `${Math.round((simState?.target_position ?? 0) / ACT_STROKE * 100)}%`, background: 'linear-gradient(90deg, #065f46, #34d399)', boxShadow: '0 0 8px #34d399' }} />
            </div>
          </div>

          {/* Stall + Last Cmd */}
          <div className="flex gap-3">
            <div className="flex-1 rounded-2xl p-5" style={{ backgroundColor: simState?.stalled ? '#1a0a0a' : '#0a0d0a', border: `1px solid ${simState?.stalled ? '#5b0a0a' : '#0e200e'}` }}>
              <span className="text-sm tracking-[0.2em] uppercase font-bold block mb-2" style={{ color: simState?.stalled ? '#f87171' : '#4ade80' }}>Stall</span>
              <span className="text-3xl font-bold" style={{ color: simState?.stalled ? '#f87171' : '#ffffff' }}>
                {simState?.stalled ? 'FAULT' : 'Clear'}
              </span>
            </div>
            <div className="flex-1 rounded-2xl p-5" style={{ backgroundColor: '#0a0d14', border: '1px solid #0e1530' }}>
              <span className="text-sm tracking-[0.2em] uppercase font-bold text-amber-400 block mb-2">Last Cmd</span>
              <span className="text-3xl font-bold text-white">{lastCommand || '—'}</span>
              {lastSource && <span className="text-xs text-slate-500 block mt-1">via {lastSource}</span>}
            </div>
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
