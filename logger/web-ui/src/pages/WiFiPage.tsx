import { useSignal, useSignalEffect } from "@preact/signals";
import { Button } from "micro-ui/widgets/button";
import { Icon } from "micro-ui/widgets/icon";
import { Input } from "micro-ui/widgets/input";
import { Spinner } from "micro-ui/widgets/spinner";
import {
  rpc,
  wifiScanning,
  wifiConnected,
  wifiSsid,
  wifiIp,
  type WiFiNetwork,
} from "../store";

export const WiFiPage = () => {
  const savedNetworks = useSignal<WiFiNetwork[]>([]);
  const availableNetworks = useSignal<WiFiNetwork[]>([]);
  const connecting = useSignal(false);
  const selectedNetwork = useSignal<WiFiNetwork | null>(null);
  const password = useSignal("");
  const showPasswordDialog = useSignal(false);
  const errorMessage = useSignal("");

  // Load status on mount
  useSignalEffect(() => {
    loadStatus();
  });

  const loadStatus = async () => {
    try {
      const status = await rpc.getStatus();
      if (status.connected) {
        // Add current network to saved list if not already there
        if (
          status.ssid &&
          !savedNetworks.value.find((n) => n.ssid === status.ssid)
        ) {
          savedNetworks.value = [
            ...savedNetworks.value,
            { ssid: status.ssid, rssi: -50, secure: true },
          ];
        }
      }
    } catch (err) {
      console.error("Failed to load status:", err);
    }
  };

  const handleScan = async () => {
    errorMessage.value = "";
    try {
      const networks = await rpc.scanNetworks();
      availableNetworks.value = networks;
    } catch (err) {
      errorMessage.value = "Failed to scan networks";
      console.error("Scan failed:", err);
    }
  };

  const handleConnect = (network: WiFiNetwork) => {
    selectedNetwork.value = network;
    password.value = "";
    showPasswordDialog.value = true;
    errorMessage.value = "";
  };

  const handleSubmitConnect = async () => {
    if (!selectedNetwork.value) return;

    connecting.value = true;
    errorMessage.value = "";
    try {
      const result = await rpc.connectToNetwork(
        selectedNetwork.value.ssid,
        password.value,
      );
      if (result.success) {
        // Add to saved networks
        if (
          !savedNetworks.value.find(
            (n) => n.ssid === selectedNetwork.value!.ssid,
          )
        ) {
          savedNetworks.value = [...savedNetworks.value, selectedNetwork.value];
        }
        showPasswordDialog.value = false;
      } else {
        errorMessage.value = result.error || "Connection failed";
      }
    } catch (err) {
      errorMessage.value = "Connection failed";
      console.error("Connect failed:", err);
    } finally {
      connecting.value = false;
    }
  };

  const handleForget = async (network: WiFiNetwork) => {
    try {
      await rpc.forgetNetwork(network.ssid);
      savedNetworks.value = savedNetworks.value.filter(
        (n) => n.ssid !== network.ssid,
      );
    } catch (err) {
      console.error("Forget failed:", err);
    }
  };

  const getSignalStrength = (rssi: number): number => {
    if (rssi >= -50) return 4;
    if (rssi >= -60) return 3;
    if (rssi >= -70) return 2;
    return 1;
  };

  const SignalBars = ({ rssi }: { rssi: number }) => {
    const strength = getSignalStrength(rssi);
    return (
      <div className="signal-bars">
        {[1, 2, 3, 4].map((bar) => (
          <div
            key={bar}
            className={`signal-bar ${bar <= strength ? "active" : ""}`}
          />
        ))}
      </div>
    );
  };

  return (
    <div className="space-y-6">
      {/* WiFi Status Card */}
      <div className="bg-panel rounded-xl p-6 border border-border">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <Icon name="wifi" size="l" />
            <div>
              <h3 className="text-lg font-semibold text-strong">WiFi Status</h3>
              {wifiConnected.value ? (
                <p className="text-sm text-muted">
                  Connected to{" "}
                  <span className="text-strong">{wifiSsid.value}</span>
                </p>
              ) : (
                <p className="text-sm text-muted">Not connected</p>
              )}
            </div>
          </div>
          <div className="text-right">
            {wifiConnected.value ? (
              <>
                <span className="status-dot connected"></span>
                <p className="text-sm font-mono text-strong">{wifiIp.value}</p>
              </>
            ) : (
              <span className="status-dot disconnected"></span>
            )}
          </div>
        </div>
      </div>

      {/* Error Message */}
      {errorMessage.value && (
        <div className="bg-error/20 border border-error rounded-lg p-4 text-error">
          {errorMessage.value}
        </div>
      )}

      {/* Saved Networks */}
      <div className="bg-panel rounded-xl p-6 border border-border">
        <h3 className="text-lg font-semibold text-strong mb-4">
          Saved Networks
        </h3>
        {savedNetworks.value.length === 0 ? (
          <p className="text-muted text-sm">No saved networks</p>
        ) : (
          <div className="space-y-2">
            {savedNetworks.value.map((network) => (
              <div
                key={network.ssid}
                className="flex items-center justify-between p-4 bg-well rounded-lg"
              >
                <div className="flex items-center gap-3">
                  <Icon name="wifi" size="m" />
                  <span className="text-sm font-medium text-strong">
                    {network.ssid}
                  </span>
                </div>
                <Button
                  variant="ghost"
                  size="s"
                  onClick={() => handleForget(network)}
                >
                  Forget
                </Button>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Available Networks */}
      <div className="bg-panel rounded-xl p-6 border border-border">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-semibold text-strong">
            Available Networks
          </h3>
          <div className="flex items-center gap-3">
            {wifiScanning.value && <Spinner size="s" />}
            <Button
              variant="outlined"
              size="s"
              iconName="refresh"
              onClick={handleScan}
              disabled={wifiScanning.value}
            >
              {wifiScanning.value ? "Scanning..." : "Scan"}
            </Button>
          </div>
        </div>

        {availableNetworks.value.length === 0 ? (
          <p className="text-muted text-sm">
            {wifiScanning.value
              ? "Scanning for networks..."
              : "Press Scan to find networks"}
          </p>
        ) : (
          <div className="space-y-2">
            {availableNetworks.value.map((network) => (
              <div
                key={network.ssid}
                className="flex items-center justify-between p-4 bg-well rounded-lg cursor-pointer hover:bg-border transition-colors"
                onClick={() => handleConnect(network)}
              >
                <div className="flex items-center gap-3">
                  <Icon name={network.secure ? "lock" : "wifi"} size="m" />
                  <span className="text-sm font-medium text-strong">
                    {network.ssid}
                  </span>
                </div>
                <SignalBars rssi={network.rssi} />
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Password Dialog */}
      {showPasswordDialog.value && selectedNetwork.value && (
        <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50 p-4">
          <div className="bg-panel rounded-xl p-6 border border-border max-w-md w-full">
            <h3 className="text-lg font-semibold text-strong mb-4">
              Connect to {selectedNetwork.value.ssid}
            </h3>

            <div className="mb-4">
              <label className="block text-sm text-muted mb-2">Password</label>
              <Input
                type="password"
                value={password.value}
                onInput={(e) =>
                  (password.value = (e.target as HTMLInputElement).value)
                }
                placeholder="Enter password"
              />
            </div>

            {errorMessage.value && (
              <div className="text-error text-sm mb-4">
                {errorMessage.value}
              </div>
            )}

            <div className="flex gap-3 justify-end">
              <Button
                variant="ghost"
                size="m"
                onClick={() => (showPasswordDialog.value = false)}
                disabled={connecting.value}
              >
                Cancel
              </Button>
              <Button
                variant="filled"
                size="m"
                onClick={handleSubmitConnect}
                disabled={connecting.value || !password.value}
              >
                {connecting.value ? (
                  <div className="flex items-center gap-2">
                    <Spinner size="xs" />
                    <span>Connecting...</span>
                  </div>
                ) : (
                  "Connect"
                )}
              </Button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};
