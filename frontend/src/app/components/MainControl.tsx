import { useState, useEffect, useRef, useCallback } from 'react';
import { Card } from './ui/card';
import { Button } from './ui/button';
import { Badge } from './ui/badge';
import { Progress } from './ui/progress';
import { Alert, AlertDescription } from './ui/alert';
import {
  Power,
  ChevronUp,
  ChevronDown,
  StopCircle,
  Activity,
  AlertTriangle,
  CheckCircle2,
  Wifi,
  WifiOff,
} from 'lucide-react';
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
}

// ---- Constants -----------------------------------------------------

const ACT_STROKE = 0.0254; // 1 inch in metres — matches simulation_stub.py
const HUB_WS_URL =
  window.location.hostname === 'localhost'
    ? 'ws://localhost:8000/ws/dashboard'
    : `ws://${window.location.hostname}:8000/ws/dashboard`;

// ---- Component -----------------------------------------------------

export function MainControl() {
  const [hubConnected, setHubConnected] = useState(false);
  const [piConnected, setPiConnected] = useState(false);
  const [simState, setSimState] = useState<SimulationState | null>(null);
  const [lastCommand, setLastCommand] = useState<string | null>(null);
  const [lastSource, setLastSource] = useState<string | null>(null);
  const [history, setHistory] = useState<HistoryEntry[]>([]);

  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null);

  // ---- WebSocket lifecycle -----------------------------------------

  const connect = useCallback(() => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) return;

    const ws = new WebSocket(HUB_WS_URL);
    wsRef.current = ws;

    ws.onopen = () => {
      setHubConnected(true);
      toast.success('Connected to Ableware Hub');
    };

    ws.onmessage = (event) => {
      try {
        const data: StateUpdate = JSON.parse(event.data);
        if (data.type === 'state_update') {
          setSimState(data.simulation_state);
          setPiConnected(data.pi_connected);
          setLastCommand(data.last_command);
          setLastSource(data.last_command_source);
          setHistory(data.command_history.slice(0, 10));
        }
      } catch {
        // ignore malformed messages
      }
    };

    ws.onerror = () => {
      toast.error('Hub connection error');
    };

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

  // ---- Send command ------------------------------------------------

  const sendCommand = (cmd: string) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      toast.error('Not connected to hub');
      return;
    }
    wsRef.current.send(
      JSON.stringify({
        type: 'command',
        command: cmd,
        timestamp: Date.now() / 1000,
        source: 'manual',
      })
    );
  };

  // ---- Derived display values -------------------------------------

  const positionPct = simState
    ? Math.round((simState.position / ACT_STROKE) * 100)
    : 0;

  const isEmergency = simState?.emergency_stopped ?? false;

  return (
    <div className="space-y-6">

      {/* Connection status */}
      <Card className="p-6">
        <div className="flex items-center justify-between mb-2">
          <h2 className="text-2xl font-semibold">Connection</h2>
          <div className="flex gap-3">
            <Badge className={`${hubConnected ? 'bg-green-500' : 'bg-gray-500'} text-white px-3 py-1`}>
              <div className="flex items-center gap-1">
                {hubConnected ? <Wifi className="w-3 h-3" /> : <WifiOff className="w-3 h-3" />}
                Hub
              </div>
            </Badge>
            <Badge className={`${piConnected ? 'bg-green-500' : 'bg-gray-500'} text-white px-3 py-1`}>
              <div className="flex items-center gap-1">
                {piConnected ? <Wifi className="w-3 h-3" /> : <WifiOff className="w-3 h-3" />}
                Pi
              </div>
            </Badge>
          </div>
        </div>
        <p className="text-sm text-gray-500">Hub: {HUB_WS_URL}</p>
      </Card>

      {/* Actuator state */}
      <Card className="p-6">
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-2xl font-semibold">Actuator State</h2>
          <Badge
            className={`px-3 py-1 text-white ${
              isEmergency
                ? 'bg-red-500'
                : simState?.at_target
                ? 'bg-green-500'
                : 'bg-yellow-500 animate-pulse'
            }`}
          >
            <div className="flex items-center gap-1">
              {isEmergency ? (
                <><AlertTriangle className="w-3 h-3" /> E-Stop</>
              ) : simState?.at_target ? (
                <><CheckCircle2 className="w-3 h-3" /> At Target</>
              ) : (
                <><Activity className="w-3 h-3" /> Moving</>
              )}
            </div>
          </Badge>
        </div>

        <div className="space-y-3">
          <div className="flex justify-between items-center">
            <span className="text-lg font-medium">Position</span>
            <span className="text-2xl font-bold text-blue-600">
              {positionPct}%
              <span className="text-sm font-normal text-gray-500 ml-2">
                ({simState ? (simState.position * 1000).toFixed(1) : '0.0'} mm)
              </span>
            </span>
          </div>
          <Progress value={positionPct} className="h-3" />

          <div className="grid grid-cols-2 gap-2 text-sm text-gray-600 mt-2">
            <span>Velocity: {simState ? (simState.velocity * 1000).toFixed(1) : '0.0'} mm/s</span>
            <span>PWM: {simState ? simState.pwm.toFixed(3) : '0.000'}</span>
            <span>Target: {simState ? (simState.target_position * 1000).toFixed(1) : '0.0'} mm</span>
            <span>Stalled: {simState?.stalled ? 'YES' : 'No'}</span>
          </div>
        </div>

        {lastCommand && (
          <div className="mt-4 p-3 bg-blue-50 rounded-lg">
            <p className="text-sm text-gray-500">Last command</p>
            <p className="text-lg font-semibold">
              {lastCommand}
              <span className="text-sm font-normal text-gray-500 ml-2">via {lastSource}</span>
            </p>
          </div>
        )}
      </Card>

      {/* Manual controls */}
      <Card className="p-6">
        <h2 className="text-2xl font-semibold mb-4">Manual Controls</h2>
        <div className="grid grid-cols-3 gap-3">
          <Button
            onClick={() => sendCommand('START')}
            disabled={!hubConnected}
            className="h-14 bg-green-600 hover:bg-green-700 col-span-3"
            size="lg"
          >
            <Power className="w-5 h-5 mr-2" /> START
          </Button>
          <Button
            onClick={() => sendCommand('UP')}
            disabled={!hubConnected || isEmergency}
            variant="outline"
            className="h-14 col-span-1"
            size="lg"
          >
            <ChevronUp className="w-5 h-5 mr-1" /> UP
          </Button>
          <div />
          <Button
            onClick={() => sendCommand('DOWN')}
            disabled={!hubConnected || isEmergency}
            variant="outline"
            className="h-14 col-span-1"
            size="lg"
          >
            <ChevronDown className="w-5 h-5 mr-1" /> DOWN
          </Button>
          <Button
            onClick={() => sendCommand('LEFT')}
            disabled={!hubConnected || isEmergency}
            variant="outline"
            className="h-14"
            size="lg"
          >
            ← LEFT
          </Button>
          <div />
          <Button
            onClick={() => sendCommand('RIGHT')}
            disabled={!hubConnected || isEmergency}
            variant="outline"
            className="h-14"
            size="lg"
          >
            RIGHT →
          </Button>
        </div>
      </Card>

      {/* Emergency stop */}
      <Card className="p-6 border-2 border-red-500">
        <h2 className="text-2xl font-semibold mb-4 text-red-600">Safety</h2>
        <Button
          onClick={() => sendCommand('STOP')}
          disabled={!hubConnected}
          className="w-full h-20 bg-red-600 hover:bg-red-700 text-xl font-bold"
          size="lg"
        >
          <AlertTriangle className="w-8 h-8 mr-3" />
          EMERGENCY STOP
        </Button>
        {isEmergency && (
          <Alert className="mt-4 border-red-400 bg-red-50">
            <StopCircle className="h-5 w-5 text-red-600" />
            <AlertDescription className="text-base text-red-700">
              Emergency stop active. Send START to resume.
            </AlertDescription>
          </Alert>
        )}
      </Card>

      {/* Command history */}
      {history.length > 0 && (
        <Card className="p-6">
          <h3 className="font-semibold text-lg mb-3">Recent Commands</h3>
          <div className="space-y-2">
            {history.map((entry, i) => (
              <div key={i} className="flex justify-between items-center text-sm p-2 bg-gray-50 rounded">
                <span className="font-medium">{entry.command}</span>
                <span className={`text-xs px-2 py-0.5 rounded ${entry.source === 'voice' ? 'bg-blue-100 text-blue-700' : 'bg-gray-200 text-gray-600'}`}>
                  {entry.source}
                </span>
                <span className="text-gray-400">
                  {new Date(entry.timestamp * 1000).toLocaleTimeString()}
                </span>
              </div>
            ))}
          </div>
        </Card>
      )}
    </div>
  );
}
