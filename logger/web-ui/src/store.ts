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
  hostUrl.value = "192.168.0.126";
}

export const wsUrl = computed(() => `ws://${hostUrl.value}/ws`);

// WebSocket connection state
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
export const downloadMode = signal(false);

// Logger config state
export interface LoggerConfig {
  config_valid: boolean;
  logging_active: boolean;
  version_major?: number;
  version_minor?: number;
  adc_sample_rate_khz?: number;
  adc_block_size?: number;
  accel_enable?: boolean;
  accel_range_g?: number;
  accel_odr_hz?: number;
  accel_bw_hz?: number;
  accel_lpf2_en?: boolean;
  accel_hp_en?: boolean;
  accel_hp_cutoff?: number;
  accel_hm_mode?: boolean;
  gyro_enable?: boolean;
  gyro_range_dps?: number;
  gyro_odr_hz?: number;
  gyro_lpf1_en?: boolean;
  gyro_lpf1_bw?: number;
  gyro_hp_en?: boolean;
  gyro_hp_cutoff?: number;
  gyro_hm_mode?: boolean;
  chip_id?: number;
  checksum_algo?: number;
  mavlink_logging_enabled?: boolean;
}

export const loggerConfig = signal<LoggerConfig | null>(null);
export const configLoading = signal(false);
export const loggingToggling = signal(false);

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
  logging_active?: boolean;
  download_mode?: boolean;
}

export interface ConnectResponse {
  success: boolean;
  ip?: string;
  error?: string;
}

// ============================================================================
// WebSocket Client
// ============================================================================

let ws: WebSocket | null = null;
let wsReconnectTimer: ReturnType<typeof setTimeout> | null = null;
let messageId = 0;
const pendingRequests = new Map<number, { 
  resolve: (value: unknown) => void; 
  reject: (reason: unknown) => void;
  timeout: ReturnType<typeof setTimeout>;
}>();

// Auto-reconnect delay (exponential backoff)
let reconnectDelay = 1000;
const MAX_RECONNECT_DELAY = 10000;

function handleStatusEvent(data: StatusResponse) {
  wifiConnected.value = data.connected;
  wifiSsid.value = data.ssid;
  wifiIp.value = data.ip;
  if (data.logging_active !== undefined) {
    loggingActive.value = data.logging_active;
  }
  if (data.download_mode !== undefined) {
    downloadMode.value = data.download_mode;
  }
  if (data.download_mode !== undefined) {
    downloadMode.value = data.download_mode;
  }
  connected.value = true;
  lastStatusTime.value = Date.now();
}

function handleWsMessage(event: MessageEvent) {
  try {
    const msg = JSON.parse(event.data);
    
    // Push event (no id)
    if (msg.event === "status" && msg.data) {
      handleStatusEvent(msg.data);
      return;
    }
    
    // Response to a request (has id)
    if (msg.id !== undefined && pendingRequests.has(msg.id)) {
      const pending = pendingRequests.get(msg.id)!;
      pendingRequests.delete(msg.id);
      clearTimeout(pending.timeout);
      pending.resolve(msg.result);
    }
  } catch (err) {
    console.error("WS message parse error:", err);
  }
}

function handleWsOpen() {
  console.log("WebSocket connected");
  connected.value = true;
  reconnectDelay = 1000; // Reset backoff on successful connect
  
  // Fetch initial config
  rpc.getConfig().catch(() => {});
}

function handleWsClose() {
  console.log("WebSocket disconnected");
  connected.value = false;
  ws = null;
  
  // Reject all pending requests
  for (const [id, pending] of pendingRequests) {
    clearTimeout(pending.timeout);
    pending.reject(new Error("WebSocket disconnected"));
  }
  pendingRequests.clear();
  
  // Schedule reconnect with exponential backoff
  if (typeof window !== "undefined") {
    wsReconnectTimer = setTimeout(() => {
      connectWebSocket();
    }, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 2, MAX_RECONNECT_DELAY);
  }
}

function handleWsError(event: Event) {
  console.error("WebSocket error:", event);
}

export function connectWebSocket() {
  if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
    return;
  }
  
  try {
    ws = new WebSocket(wsUrl.value);
    ws.onopen = handleWsOpen;
    ws.onclose = handleWsClose;
    ws.onerror = handleWsError;
    ws.onmessage = handleWsMessage;
  } catch (err) {
    console.error("WebSocket connect error:", err);
    handleWsClose();
  }
}

export function disconnectWebSocket() {
  if (wsReconnectTimer) {
    clearTimeout(wsReconnectTimer);
    wsReconnectTimer = null;
  }
  if (ws) {
    ws.onclose = null; // Prevent auto-reconnect
    ws.close();
    ws = null;
  }
  connected.value = false;
}

// Send command via WebSocket, returns Promise
function wsSend<T>(cmd: string, params?: Record<string, unknown>, timeout = 5000): Promise<T> {
  return new Promise((resolve, reject) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      reject(new Error("WebSocket not connected"));
      return;
    }
    
    const id = ++messageId;
    const timeoutHandle = setTimeout(() => {
      pendingRequests.delete(id);
      reject(new Error(`WS request timeout: ${cmd}`));
    }, timeout);
    
    pendingRequests.set(id, { 
      resolve: resolve as (value: unknown) => void, 
      reject, 
      timeout: timeoutHandle 
    });
    
    const msg = params ? { id, cmd, ...params } : { id, cmd };
    ws.send(JSON.stringify(msg));
  });
}

// ============================================================================
// RPC API (via WebSocket)
// ============================================================================

export const rpc = {
  async scanNetworks(): Promise<WiFiNetwork[]> {
    wifiScanning.value = true;
    try {
      const result = await wsSend<{ success: boolean; networks?: WiFiNetwork[] }>("scan", undefined, 15000);
      return result.networks || [];
    } finally {
      wifiScanning.value = false;
    }
  },

  async connectToNetwork(ssid: string, password: string): Promise<ConnectResponse> {
    loading.value = true;
    try {
      const result = await wsSend<ConnectResponse>("connect", { ssid, password }, 15000);
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
    return wsSend<{ success: boolean }>("forget", { ssid });
  },

  async getStatus(): Promise<StatusResponse> {
    const result = await wsSend<StatusResponse>("status", undefined, 3000);
    handleStatusEvent(result);
    return result;
  },

  async getConfig(): Promise<LoggerConfig> {
    configLoading.value = true;
    try {
      const result = await wsSend<LoggerConfig>("get_config", undefined, 3000);
      loggerConfig.value = result;
      loggingActive.value = result.logging_active;
      return result;
    } catch (err) {
      loggerConfig.value = null;
      throw err;
    } finally {
      configLoading.value = false;
    }
  },

  async setConfig(config: Partial<LoggerConfig>): Promise<LoggerConfig> {
    const result = await wsSend<{ success: boolean; error?: string; config?: LoggerConfig }>(
      "set_config",
      { config },
      5000,
    );
    if (!result.success) {
      throw new Error(result.error || "Failed to save config");
    }
    if (result.config) {
      loggerConfig.value = result.config;
      loggingActive.value = result.config.logging_active;
      return result.config;
    }
    return rpc.getConfig();
  },

  async startLogging(): Promise<{ success: boolean }> {
    loggingToggling.value = true;
    try {
      const result = await wsSend<{ success: boolean }>("start_logging", undefined, 5000);
      if (result.success) {
        loggingActive.value = true;
      }
      return result;
    } finally {
      loggingToggling.value = false;
    }
  },

  async stopLogging(): Promise<{ success: boolean }> {
    loggingToggling.value = true;
    try {
      const result = await wsSend<{ success: boolean }>("stop_logging", undefined, 5000);
      if (result.success) {
        loggingActive.value = false;
      }
      return result;
    } finally {
      loggingToggling.value = false;
    }
  },

  async mountStorage(): Promise<{ success: boolean }> {
    return wsSend<{ success: boolean }>("mount_storage");
  },

  async unmountStorage(): Promise<{ success: boolean }> {
    return wsSend<{ success: boolean }>("unmount_storage");
  },
};

// ============================================================================
// Auto-connect WebSocket on load
// ============================================================================

if (typeof window !== "undefined") {
  connectWebSocket();
}

// ============================================================================
// Navigation
// ============================================================================

const getHash = () =>
  typeof window !== "undefined" ? window.location.hash.slice(1) || "home" : "home";

currentPage.value = getHash() === "wifi" ? "wifi" : "home";

if (typeof window !== "undefined") {
  window.addEventListener("hashchange", () => {
    const hash = getHash();
    currentPage.value = hash === "wifi" ? "wifi" : "home";
  });
}

export function navigateTo(page: "home" | "wifi") {
  if (typeof window !== "undefined") {
    window.location.hash = page === "home" ? "" : page;
  }
  menuOpen.value = false;
}

// Legacy exports
export const connectionStatus = computed(() =>
  wifiScanning.value
    ? "scanning"
    : wifiConnected.value
      ? "connected"
      : "disconnected",
);

export const deviceIp = computed(() => wifiIp.value || "--");
