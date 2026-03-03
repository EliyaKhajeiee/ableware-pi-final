import { MainControl } from './components/MainControl';
import { Toaster } from './components/ui/sonner';

export default function App() {
  return (
    <div className="min-h-screen bg-gradient-to-br from-blue-50 to-indigo-100 p-4 md:p-8">
      <div className="max-w-4xl mx-auto">
        {/* Header */}
        <div className="mb-8 text-center">
          <h1 className="text-4xl md:text-5xl font-bold text-gray-900 mb-2">
            Ableware
          </h1>
          <p className="text-lg text-gray-600">
            Voice-Controlled Assistive Robotic System
          </p>
        </div>

        {/* Main Control Interface */}
        <MainControl />
      </div>
      <Toaster />
    </div>
  );
}