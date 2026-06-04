import { MainControl } from './components/MainControl';
import { Toaster } from './components/ui/sonner';

export default function App() {
  return (
    <div className="min-h-screen bg-[#080810] text-[#e2e8f0]">
      <MainControl />
      <Toaster theme="dark" />
    </div>
  );
}
