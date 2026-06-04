import { MainControl } from './components/MainControl';
import { Toaster } from './components/ui/sonner';

export default function App() {
  return (
    <div className="min-h-screen bg-[#05050a] text-white overflow-hidden">
      <MainControl />
      <Toaster theme="dark" position="top-right" />
    </div>
  );
}
