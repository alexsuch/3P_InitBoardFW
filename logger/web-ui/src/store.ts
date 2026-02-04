import { signal, computed, Signal } from "@preact/signals";

// Development mode detection
export const isDev = signal(
  typeof import.meta !== "undefined" &&
    import.meta.env?.MODE === "development",
);

// Host/Server URLs - in dev mode use hardcoded IP
export const hostUrl: Signal<string> = signal(
  typeof window !== "undefined" ? window.location.host : "localhost",
);

// For development: connect to device at this IP
if (isDev.value) {
  hostUrl.value = "192.168.0.109";
}

export const serverUrl = computed(() => `http://${hostUrl.value}`);
export const apiUrl = computed(() => `${serverUrl.value}/rpc`);

// Connection state - true when we can reach the logger
export const connected = signal(false);
export const loading = signal(false);
export const lastStatusTime = signal<number>(0);

// WiFi state
export const wifiConnected = signal(false);
export const wifiSsid = signal<string | undefined>(undefined);
export const wifiIp = signal<string | undefined>(undefined);
export const wifiScanning = signal(false);

// Navigation state
export const currentPage = signal<"home" | "wifi">("home");
export const menuOpen = signal(false);

// Device state
export const deviceVersion = signal<string | undefined>(undefined);
export const sdCardStatus = signal<"ok" | "error" | "missing">("missing");
export const loggingActive = signal(false);

// RPC types
export interface WiFiNetwork {
  ssid: string;
  rssi: number;
  secure: boolean;
}

export interface StatusResponse {
  connected: boolean;
  ssid?: string;
  ip?: string;
  scanning?: boolean;
}

export interface ConnectResponse {
  success: boolean;
  ip?: string;
  error?: string;
}

// RPC client with timeout
async function rpcCall<T>(
  cmd: string,
  params?: Record<string, unknown>,
  timeout = 2000,
): Promise<T> {
  const body = params ? { cmd, ...params } : { cmd };
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(apiUrl.value, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(body),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      throw new Error(`RPC call failed: ${response.status}`);
    }

    return response.json();
  } catch (err) {
    clearTimeout(timeoutId);
    throw err;
  }
}

// RPC API functions
export const rpc = {
  async scanNetworks(): Promise<WiFiNetwork[]> {
    wifiScanning.value = true;
    try {
      const result = await rpcCall<{ networks: WiFiNetwork[] }>("scan", undefined, 15000);
      return result.networks || [];
    } finally {
      wifiScanning.value = false;
    }
  },

  async connectToNetwork(
    ssid: string,
    password: string,
  ): Promise<ConnectResponse> {
    loading.value = true;
    try {
      const result = await rpcCall<ConnectResponse>("connect", {
        ssid,
        password,
      }, 10000);
      if (result.success) {
        wifiConnected.value = true;
        wifiSsid.value = ssid;
        wifiIp.value = result.ip;
      }
      return result;
    } finally {
      loading.value = false;
    }
  },

  async forgetNetwork(ssid: string): Promise<{ success: boolean }> {
    return rpcCall<{ success: boolean }>("forget", { ssid });
  },

  // Status polling - called frequently to maintain connection state
  async getStatus(): Promise<StatusResponse> {
    try {
      const result = await rpcCall<StatusResponse>("status", undefined, 2000);
      wifiConnected.value = result.connected;
      wifiSsid.value = result.ssid;
      wifiIp.value = result.ip;
      if (result.scanning !== undefined) {
        wifiScanning.value = result.scanning;
      }
      connected.value = true;
      lastStatusTime.value = Date.now();
      return result;
    } catch (err) {
      connected.value = false;
      throw err;
    }
  },
};

// Status polling interval
let statusIntervalId: ReturnType<typeof setInterval> | null = null;
const STATUS_POLL_INTERVAL = 500; // Poll every 500ms
const STATUS_TIMEOUT = 3000; // Consider disconnected after 3s without response

export function startStatusPolling() {
  if (statusIntervalId) return;

  // Initial status fetch
  rpc.getStatus().catch(() => {});

  statusIntervalId = setInterval(() => {
    // Check if we haven't received status in a while
    if (lastStatusTime.value > 0 && Date.now() - lastStatusTime.value > STATUS_TIMEOUT) {
      connected.value = false;
    }
    
    // Fetch status
    rpc.getStatus().catch(() => {
      // Silently fail - connection status is handled by timeout
    });
  }, STATUS_POLL_INTERVAL);
}

export function stopStatusPolling() {
  if (statusIntervalId) {
    clearInterval(statusIntervalId);
    statusIntervalId = null;
  }
}

// Start polling when module loads (in browser only)
if (typeof window !== "undefined") {
  startStatusPolling();
}

// Hash-based routing
const getHash = () =>
  typeof window !== "undefined" ? window.location.hash.slice(1) || "home" : "home";

// Initialize current page from hash
currentPage.value = getHash() === "wifi" ? "wifi" : "home";

// Listen for hash changes
if (typeof window !== "undefined") {
  window.addEventListener("hashchange", () => {
    const hash = getHash();
    currentPage.value = hash === "wifi" ? "wifi" : "home";
  });
}

// Navigation helper
export function navigateTo(page: "home" | "wifi") {
  if (typeof window !== "undefined") {
    window.location.hash = page === "home" ? "" : page;
  }
  menuOpen.value = false;
}

// Legacy exports for backward compatibility
export const connectionStatus = computed(() =>
  wifiScanning.value
    ? "scanning"
    : wifiConnected.value
      ? "connected"
      : "disconnected",
);

export const deviceIp = computed(() => wifiIp.value || "--");
