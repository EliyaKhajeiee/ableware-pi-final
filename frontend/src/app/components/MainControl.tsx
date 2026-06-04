import { useState, useEffect, useRef, useCallback } from 'react';
import { ChevronUp, ChevronDown, Power, AlertTriangle, Activity, Wifi, WifiOff, Circle } from 'lucide-react';
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

// ---- Sub-components ------------------------------------------------

function StatusDot({ active, color, label }: { active: boolean; color: string; label: string }) {
  return (
    <div className="flex items-center gap-1.5">
      <div className={`w-2 h-2 rounded-full transition-all duration-500 ${active ? color : 'bg-[#2a2a3a]'} ${active ? 'shadow-[0_0_6px_currentColor]' : ''}`} />
      <span className={`text-xs font-mono tracking-widest uppercase ${active ? 'text-[#94a3b8]' : 'text-[#3a3a4a]'}`}>
        {label}
      </span>
    </div>
  );
}

function MetricRow({ label, value, unit }: { label: string; value: string; unit?: string }) {
  return (
    <div className="flex items-center justify-between py-2 border-b border-[#1a1a2e]">
      <span className="text-xs font-mono tracking-widest uppercase text-[#4a5568]">{label}</span>
      <span className="text-sm font-mono text-[#94a3b8]">
        {value}
        {unit && <span className="text-[#4a5568] ml-1">{unit}</span>}
      </span>
    </div>
  );
}

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
          setHistory(data.command_history.slice(0, 8));
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
    <div className="min-h-screen bg-[#080810] flex flex-col">

      {/* ── Header ───────────────────────────────────────────── */}
      <header className="border-b border-[#1a1a2e] px-8 py-4 flex items-center justify-between">
        <div>
          <h1 className="text-xl font-mono font-bold tracking-[0.2em] uppercase text-[#e2e8f0]">
            Ableware
          </h1>
          <p className="text-xs font-mono text-[#3a3a5a] tracking-widest mt-0.5">
            ASSISTIVE LIFT CONTROL SYSTEM
          </p>
        </div>

        <div className="flex items-center gap-6">
          <StatusDot active={hubConnected} color="bg-emerald-400 text-emerald-400" label="Hub" />
          <StatusDot active={piConnected} color="bg-violet-400 text-violet-400" label="Pi" />
          <StatusDot active={simConnected} color="bg-sky-400 text-sky-400" label="Sim" />
          <StatusDot active={arduinoConnected} color="bg-cyan-400 text-cyan-400" label="Arduino" />
        </div>
      </header>

      {/* ── Main content ─────────────────────────────────────── */}
      <div className="flex-1 grid grid-cols-1 lg:grid-cols-2 gap-px bg-[#1a1a2e]">

        {/* ── Left panel: Telemetry ────────────────────────── */}
        <div className="bg-[#080810] p-8 flex flex-col gap-6">

          <div className="flex items-center justify-between">
            <span className="text-xs font-mono tracking-widest uppercase text-[#4a5568]">
              Actuator Telemetry
            </span>
            <div className={`flex items-center gap-1.5 text-xs font-mono ${
              isEmergency ? 'text-red-400' : isMoving ? 'text-amber-400' : 'text-emerald-400'
            }`}>
              <Circle className={`w-2 h-2 fill-current ${isMoving && !isEmergency ? 'animate-pulse' : ''}`} />
              {isEmergency ? 'E-STOP' : isMoving ? 'MOVING' : 'STABLE'}
            </div>
          </div>

          {/* Position bar */}
          <div className="space-y-3">
            <div className="flex justify-between items-baseline">
              <span className="text-xs font-mono tracking-widest uppercase text-[#4a5568]">Position</span>
              <div className="text-right">
                <span className="text-3xl font-mono font-bold text-[#e2e8f0]">{positionPct}</span>
                <span className="text-lg font-mono text-[#4a5568] ml-1">%</span>
                <span className="text-xs font-mono text-[#3a3a5a] ml-2">
                  {simState ? (simState.position * 1000).toFixed(1) : '0.0'} mm
                </span>
              </div>
            </div>
            <div className="h-1.5 bg-[#1a1a2e] rounded-full overflow-hidden">
              <div
                className="h-full bg-gradient-to-r from-sky-600 to-sky-400 rounded-full transition-all duration-500"
                style={{ width: `${positionPct}%` }}
              />
            </div>
          </div>

          {/* Step indicator */}
          <div className="space-y-3">
            <div className="flex justify-between items-baseline">
              <span className="text-xs font-mono tracking-widest uppercase text-[#4a5568]">Step Position</span>
              <span className="text-sm font-mono text-[#94a3b8]">
                <span className="text-[#e2e8f0]">{positionSteps}</span>
                <span className="text-[#3a3a5a]"> / 7</span>
              </span>
            </div>
            <div className="h-1 bg-[#1a1a2e] rounded-full overflow-hidden">
              <div
                className="h-full bg-gradient-to-r from-violet-600 to-violet-400 rounded-full transition-all duration-300"
                style={{ width: `${stepPct}%` }}
              />
            </div>
            <div className="flex justify-between">
              {Array.from({ length: 8 }, (_, i) => (
                <div
                  key={i}
                  className={`w-1.5 h-1.5 rounded-full transition-all duration-300 ${
                    i <= positionSteps ? 'bg-violet-400' : 'bg-[#1a1a2e]'
                  }`}
                />
              ))}
            </div>
          </div>

          {/* Metrics */}
          <div className="flex-1 border border-[#1a1a2e] rounded-lg p-4 space-y-0">
            <MetricRow label="Velocity" value={simState ? (simState.velocity * 1000).toFixed(1) : '—'} unit="mm/s" />
            <MetricRow label="PWM Output" value={simState ? simState.pwm.toFixed(3) : '—'} />
            <MetricRow label="Target" value={simState ? (simState.target_position * 1000).toFixed(1) : '—'} unit="mm" />
            <MetricRow label="Stall" value={simState?.stalled ? 'DETECTED' : 'Clear'} />
            {lastCommand && (
              <MetricRow label="Last Cmd" value={`${lastCommand} via ${lastSource}`} />
            )}
          </div>
        </div>

        {/* ── Right panel: Controls ────────────────────────── */}
        <div className="bg-[#080810] p-8 flex flex-col gap-4">

          <span className="text-xs font-mono tracking-widest uppercase text-[#4a5568]">
            Manual Control
          </span>

          {/* Resume */}
          <button
            onClick={() => sendCommand('START')}
            disabled={!hubConnected}
            className={`w-full py-4 rounded-lg border font-mono text-sm tracking-widest uppercase transition-all duration-200 ${
              hubConnected
                ? 'border-emerald-800 text-emerald-400 hover:bg-emerald-950 hover:border-emerald-600'
                : 'border-[#1a1a2e] text-[#2a2a3a] cursor-not-allowed'
            }`}
          >
            <Power className="w-4 h-4 inline mr-2" />
            Resume
          </button>

          {/* UP */}
          <button
            onClick={() => sendCommand('UP')}
            disabled={!hubConnected || isEmergency}
            className={`flex-1 w-full py-10 rounded-lg border font-mono text-lg tracking-widest uppercase transition-all duration-200 ${
              hubConnected && !isEmergency
                ? 'border-sky-800 text-sky-300 hover:bg-sky-950 hover:border-sky-500 active:scale-[0.98]'
                : 'border-[#1a1a2e] text-[#2a2a3a] cursor-not-allowed'
            }`}
          >
            <ChevronUp className="w-8 h-8 mx-auto mb-1" />
            Up
          </button>

          {/* DOWN */}
          <button
            onClick={() => sendCommand('DOWN')}
            disabled={!hubConnected || isEmergency}
            className={`flex-1 w-full py-10 rounded-lg border font-mono text-lg tracking-widest uppercase transition-all duration-200 ${
              hubConnected && !isEmergency
                ? 'border-sky-800 text-sky-300 hover:bg-sky-950 hover:border-sky-500 active:scale-[0.98]'
                : 'border-[#1a1a2e] text-[#2a2a3a] cursor-not-allowed'
            }`}
          >
            <ChevronDown className="w-8 h-8 mx-auto mb-1" />
            Down
          </button>

          {/* Emergency stop */}
          <button
            onClick={() => sendCommand('STOP')}
            disabled={!hubConnected}
            className={`w-full py-5 rounded-lg border-2 font-mono text-base tracking-widest uppercase transition-all duration-200 ${
              isEmergency
                ? 'border-red-500 bg-red-950 text-red-300 animate-pulse'
                : hubConnected
                ? 'border-red-900 text-red-500 hover:bg-red-950 hover:border-red-600'
                : 'border-[#1a1a2e] text-[#2a2a3a] cursor-not-allowed'
            }`}
          >
            <AlertTriangle className="w-4 h-4 inline mr-2" />
            {isEmergency ? 'E-STOP ACTIVE' : 'Emergency Stop'}
          </button>
        </div>
      </div>

      {/* ── Command log ──────────────────────────────────────── */}
      {history.length > 0 && (
        <div className="border-t border-[#1a1a2e] px-8 py-4">
          <div className="flex items-center gap-3 mb-3">
            <Activity className="w-3 h-3 text-[#3a3a5a]" />
            <span className="text-xs font-mono tracking-widest uppercase text-[#3a3a5a]">Command Log</span>
          </div>
          <div className="flex gap-6 overflow-x-auto">
            {history.map((entry, i) => (
              <div key={i} className={`flex-shrink-0 font-mono text-xs ${i === 0 ? 'text-[#94a3b8]' : 'text-[#3a3a5a]'}`}>
                <span className="text-[#2a2a3a] mr-2">
                  {new Date(entry.timestamp * 1000).toLocaleTimeString('en', { hour12: false })}
                </span>
                <span className={`mr-2 ${
                  entry.source === 'voice' ? 'text-violet-400' :
                  entry.source === 'manual' ? 'text-sky-400' : 'text-[#94a3b8]'
                }`}>{entry.command}</span>
                <span className="text-[#2a2a3a]">{entry.source}</span>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
