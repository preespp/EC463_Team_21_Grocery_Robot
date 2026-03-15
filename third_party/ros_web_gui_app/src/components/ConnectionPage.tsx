import { useState, useEffect } from 'react';
import './ConnectionPage.css';
import { loadConnectionPreferences, saveConnectionPreferences } from '../utils/connectionPreferences';

interface ConnectionPageProps {
  onConnect: (url: string) => Promise<boolean>;
}

export function ConnectionPage({ onConnect }: ConnectionPageProps) {
  const [ip, setIp] = useState(() => {
    const preferences = loadConnectionPreferences();
    if (preferences?.ip) {
      return preferences.ip;
    }
    const hostname = window.location.hostname;
    return hostname || 'localhost';
  });
  const [port, setPort] = useState(() => {
    const preferences = loadConnectionPreferences();
    return preferences?.port || '9090';
  });
  const [connecting, setConnecting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    saveConnectionPreferences({ ip, port });
  }, [ip, port]);

  const handleConnect = async () => {
    if (!ip || !port) {
      setError('Enter both host and port');
      return;
    }

    setConnecting(true);
    setError(null);

    try {
      const url = `ws://${ip}:${port}`;
      const success = await onConnect(url);
      if (!success) {
        setError('Connection failed. Check the host and port.');
      } else {
        saveConnectionPreferences({ ip, port });
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Connection failed');
    } finally {
      setConnecting(false);
    }
  };

  return (
    <div className="ConnectionPage">
      <div className="ConnectionForm">
        <h1>Robot Connection</h1>
        <div className="FormGroup">
          <label htmlFor="ip">Host:</label>
          <input
            id="ip"
            type="text"
            value={ip}
            onChange={(e) => setIp(e.target.value)}
            placeholder="localhost"
            disabled={connecting}
          />
        </div>
        <div className="FormGroup">
          <label htmlFor="port">Port:</label>
          <input
            id="port"
            type="text"
            value={port}
            onChange={(e) => setPort(e.target.value)}
            placeholder="9090"
            disabled={connecting}
          />
        </div>
        {error && <div className="Error">{error}</div>}
        <button onClick={handleConnect} disabled={connecting}>
          {connecting ? 'Connecting...' : 'Connect'}
        </button>
      </div>
    </div>
  );
}


