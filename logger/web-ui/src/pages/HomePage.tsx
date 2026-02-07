import { useSignal, useSignalEffect } from "@preact/signals";
import { Button } from "micro-ui/widgets/button";
import { Icon } from "micro-ui/widgets/icon";
import { Select } from "micro-ui/widgets/select";
import { Switch } from "micro-ui/widgets/switch";
import { Spinner } from "micro-ui/widgets/spinner";
import { toastService } from "micro-ui/widgets/toast-service";
import type { LoggerConfig } from "../store";
import {
  rpc,
  loggerConfig,
  loggingActive,
  configLoading,
  loggingToggling,
  connected,
  downloadMode,
} from "../store";

// Option definitions for selects - using name/value format for micro-ui
const ACCEL_RANGE_OPTIONS = [
  { name: "±2g", value: "2" },
  { name: "±4g", value: "4" },
  { name: "±8g", value: "8" },
  { name: "±16g", value: "16" },
];

const GYRO_RANGE_OPTIONS = [
  { name: "±125 dps", value: "125" },
  { name: "±245 dps", value: "245" },
  { name: "±500 dps", value: "500" },
  { name: "±1000 dps", value: "1000" },
  { name: "±2000 dps", value: "2000" },
];

const ODR_OPTIONS = [
  { name: "Off", value: "0" },
  { name: "12.5 Hz", value: "13" },
  { name: "26 Hz", value: "26" },
  { name: "52 Hz", value: "52" },
  { name: "104 Hz", value: "104" },
  { name: "208 Hz", value: "208" },
  { name: "416 Hz", value: "416" },
  { name: "833 Hz", value: "833" },
  { name: "1.66 kHz", value: "1660" },
  { name: "3.33 kHz", value: "3330" },
  { name: "6.66 kHz", value: "6660" },
];

const ACCEL_BW_OPTIONS = [
  { name: "50 Hz", value: "50" },
  { name: "100 Hz", value: "100" },
  { name: "200 Hz", value: "200" },
  { name: "400 Hz", value: "400" },
];

const HP_CUTOFF_OPTIONS = [
  { name: "ODR/50", value: "0" },
  { name: "ODR/100", value: "1" },
  { name: "ODR/9", value: "2" },
  { name: "ODR/400", value: "3" },
];

const GYRO_LPF1_BW_OPTIONS = [
  { name: "FTYPE 0", value: "0" },
  { name: "FTYPE 1", value: "1" },
  { name: "FTYPE 2", value: "2" },
  { name: "FTYPE 3", value: "3" },
];

export const HomePage = () => {
  const error = useSignal("");
  const saving = useSignal(false);

  // Load config on mount
  useSignalEffect(() => {
    if (connected.value && !loggerConfig.value) {
      loadConfig();
    }
  });

  const loadConfig = async () => {
    error.value = "";
    try {
      await rpc.getConfig();
    } catch (err) {
      error.value = "Failed to load configuration";
      console.error("Failed to load config:", err);
    }
  };

  const handleToggleLogging = async () => {
    error.value = "";
    try {
      if (loggingActive.value) {
        await rpc.stopLogging();
      } else {
        await rpc.startLogging();
      }
    } catch (err) {
      error.value = "Failed to toggle logging";
      console.error("Toggle logging failed:", err);
    }
  };

  const config = loggerConfig.value;
  const isLogging = loggingActive.value;
  const isDownload = downloadMode.value;
  const isDisabled = isLogging || isDownload || !config?.config_valid;

  const toInt = (v: unknown, fallback: number) => {
    const n = typeof v === "number" ? v : parseInt(String(v), 10);
    return Number.isFinite(n) ? n : fallback;
  };

  const updateConfig = (patch: Partial<LoggerConfig>) => {
    const current = loggerConfig.value;
    if (!current) return;
    loggerConfig.value = { ...current, ...patch };
  };

  // Find closest value for select matching
  const findClosestOdr = (hz: number | undefined) => {
    if (!hz) return "0";
    const options = ODR_OPTIONS.map((o) => parseInt(o.value));
    const closest = options.reduce((prev, curr) =>
      Math.abs(curr - hz) < Math.abs(prev - hz) ? curr : prev,
    );
    return closest.toString();
  };

  const handleSaveConfig = async () => {
    error.value = "";
    const cfg = loggerConfig.value;
    if (!cfg?.config_valid) return;

    saving.value = true;
    try {
      await rpc.setConfig({
        accel_enable: cfg.accel_enable ?? false,
        accel_range_g: cfg.accel_range_g ?? 16,
        accel_odr_hz: toInt(findClosestOdr(cfg.accel_odr_hz), 0),
        accel_bw_hz: cfg.accel_bw_hz ?? 400,
        accel_lpf2_en: cfg.accel_lpf2_en ?? false,
        accel_hp_en: cfg.accel_hp_en ?? false,
        accel_hp_cutoff: cfg.accel_hp_cutoff ?? 0,
        accel_hm_mode: cfg.accel_hm_mode ?? false,

        gyro_enable: cfg.gyro_enable ?? false,
        gyro_range_dps: cfg.gyro_range_dps ?? 2000,
        gyro_odr_hz: toInt(findClosestOdr(cfg.gyro_odr_hz), 0),
        gyro_lpf1_en: cfg.gyro_lpf1_en ?? false,
        gyro_lpf1_bw: cfg.gyro_lpf1_bw ?? 0,
        gyro_hp_en: cfg.gyro_hp_en ?? false,
        gyro_hp_cutoff: cfg.gyro_hp_cutoff ?? 0,
        gyro_hm_mode: cfg.gyro_hm_mode ?? false,
      });
      toastService.showSuccess("Configuration saved");
    } catch (err) {
      error.value = "Failed to save configuration";
      console.error("Save config failed:", err);
    } finally {
      saving.value = false;
    }
  };

  return (
    <div className="space-y-6 max-w-3xl mx-auto">
      {/* Status Banner */}
      {!connected.value && (
        <div className="bg-warning/10 border border-warning rounded-lg p-4 flex items-center gap-3">
          <Icon name="warning" />
          <span className="text-warning">
            Logger not connected. Make sure the device is powered on.
          </span>
        </div>
      )}

      {error.value && (
        <div className="bg-error/10 border border-error rounded-lg p-4 flex items-center gap-3">
          <Icon name="warning" />
          <span className="text-error">{error.value}</span>
        </div>
      )}

      {/* Logging Control Card */}
      <div className="bg-panel border border-border rounded-xl p-6">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <div
              className={`w-4 h-4 rounded-full ${isLogging ? "bg-success animate-pulse" : "bg-muted"}`}
            />
            <div>
              <h3 className="text-lg font-semibold text-strong">
                {isLogging ? "Recording Active" : "Ready to Record"}
              </h3>
              <p className="text-sm text-muted">
                {isLogging
                  ? "Data is being recorded to SD card"
                  : "Press Start to begin logging"}
              </p>
            </div>
          </div>

          <Button
            variant={isLogging ? "outlined" : "filled"}
            onClick={handleToggleLogging}
            disabled={loggingToggling.value || !connected.value || isDownload}
          >
            {loggingToggling.value ? (
              <Spinner size="s" />
            ) : isLogging ? (
              <>
                <Icon name="stop" size="s" />
                Stop Logging
              </>
            ) : (
              <>
                <Icon name="play" size="s" />
                Start Logging
              </>
            )}
          </Button>
        </div>
      </div>

      {/* Storage Control Card */}
      <div className="bg-panel border border-border rounded-xl p-6">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-4">
            <Icon
              name="sd-card"
              size="l"
              className={isDownload ? "text-primary" : "text-muted"}
            />
            <div>
              <h3 className="text-lg font-semibold text-strong">
                File Storage
              </h3>
              <p className="text-sm text-muted">
                {isDownload
                  ? "Mounted as USB Mass Storage"
                  : "Mount to access files via USB"}
              </p>
            </div>
          </div>

          <Button
            variant={isDownload ? "outlined" : "outlined"}
            onClick={async () => {
              if (isDownload) {
                await rpc.unmountStorage();
              } else {
                await rpc.mountStorage();
              }
            }}
            disabled={isLogging || !connected.value}
          >
            {isDownload ? (
              <>
                <Icon name="stop" size="s" />
                Unmount Storage
              </>
            ) : (
              <>
                <Icon name="download" size="s" />
                Mount Storage
              </>
            )}
          </Button>
        </div>
      </div>

      {/* Config Loading State */}
      {configLoading.value && (
        <div className="bg-panel border border-border rounded-xl p-12 flex flex-col items-center justify-center gap-4">
          <Spinner size="l" />
          <p className="text-muted">Loading configuration from logger...</p>
        </div>
      )}

      {/* Configuration Form */}
      {config?.config_valid && !configLoading.value && (
        <div className="bg-panel border border-border rounded-xl overflow-hidden">
          <div className="px-6 py-4 border-b border-border flex items-center justify-between">
            <div className="flex items-center gap-3">
              <Icon name="settings" />
              <h3 className="text-lg font-semibold text-strong">
                Sensor Configuration
              </h3>
            </div>
            <span className="text-xs text-muted bg-well px-2 py-1 rounded">
              v{config.version_major}.{config.version_minor}
            </span>
          </div>

          {(isLogging || isDownload) && (
            <div className="px-6 py-3 bg-warning/10 border-b border-warning/30 flex items-center gap-2">
              <Icon name="warning" size="s" />
              <span className="text-sm text-warning">
                Configuration is locked while{" "}
                {isLogging ? "logging is active" : "storage is mounted"}
              </span>
            </div>
          )}

          <div className="p-6 space-y-8">
            {/* ADC Settings */}
            <section>
              <h4 className="text-sm font-semibold text-muted uppercase tracking-wider mb-4">
                ADC Settings
              </h4>
              <div className="grid grid-cols-2 gap-4">
                <div className="bg-well rounded-lg p-4">
                  <label className="text-xs text-muted block mb-1">
                    Sample Rate
                  </label>
                  <p className="text-lg font-medium text-strong">
                    {config.adc_sample_rate_khz} kHz
                  </p>
                </div>
                <div className="bg-well rounded-lg p-4">
                  <label className="text-xs text-muted block mb-1">
                    Block Size
                  </label>
                  <p className="text-lg font-medium text-strong">
                    {config.adc_block_size} samples
                  </p>
                </div>
              </div>
            </section>

            {/* Accelerometer Settings */}
            <section>
              <div className="flex items-center justify-between mb-4">
                <h4 className="text-sm font-semibold text-muted uppercase tracking-wider">
                  Accelerometer
                </h4>
                <Switch
                  checked={config.accel_enable ?? false}
                  disabled={isDisabled}
                  onChange={(checked) => updateConfig({ accel_enable: !!checked })}
                  label=""
                />
              </div>

              <div className="grid grid-cols-2 lg:grid-cols-3 gap-4">
                <div>
                  <label className="text-xs text-muted block mb-2">Range</label>
                  <Select
                    value={config.accel_range_g?.toString() ?? "16"}
                    options={ACCEL_RANGE_OPTIONS}
                    disabled={isDisabled}
                    onChange={(v) =>
                      updateConfig({ accel_range_g: toInt(v, config.accel_range_g ?? 16) })
                    }
                  />
                </div>
                <div>
                  <label className="text-xs text-muted block mb-2">
                    Output Data Rate
                  </label>
                  <Select
                    value={findClosestOdr(config.accel_odr_hz)}
                    options={ODR_OPTIONS}
                    disabled={isDisabled}
                    onChange={(v) =>
                      updateConfig({ accel_odr_hz: toInt(v, config.accel_odr_hz ?? 0) })
                    }
                  />
                </div>
                <div>
                  <label className="text-xs text-muted block mb-2">
                    Anti-Alias Bandwidth
                  </label>
                  <Select
                    value={config.accel_bw_hz?.toString() ?? "400"}
                    options={ACCEL_BW_OPTIONS}
                    disabled={isDisabled}
                    onChange={(v) =>
                      updateConfig({ accel_bw_hz: toInt(v, config.accel_bw_hz ?? 400) })
                    }
                  />
                </div>
              </div>

              {/* Advanced Filters */}
              <div className="mt-4 pt-4 border-t border-border/50">
                <p className="text-xs text-muted mb-3">Advanced Filters</p>
                <div className="grid grid-cols-2 lg:grid-cols-3 gap-4">
                  <div className="flex items-center justify-between bg-well rounded-lg px-4 py-3">
                    <span className="text-sm">LPF2 Enable</span>
                    <Switch
                      checked={config.accel_lpf2_en ?? false}
                      disabled={isDisabled}
                      onChange={(checked) => updateConfig({ accel_lpf2_en: !!checked })}
                      label=""
                    />
                  </div>
                  <div className="flex items-center justify-between bg-well rounded-lg px-4 py-3">
                    <span className="text-sm">HP Filter</span>
                    <Switch
                      checked={config.accel_hp_en ?? false}
                      disabled={isDisabled}
                      onChange={(checked) => updateConfig({ accel_hp_en: !!checked })}
                      label=""
                    />
                  </div>
                  <div className="flex items-center justify-between bg-well rounded-lg px-4 py-3">
                    <span className="text-sm">Low-Power/Normal Mode</span>
                    <Switch
                      checked={config.accel_hm_mode ?? false}
                      disabled={isDisabled}
                      onChange={(checked) => updateConfig({ accel_hm_mode: !!checked })}
                      label=""
                    />
                  </div>
                  {config.accel_hp_en && (
                    <div>
                      <label className="text-xs text-muted block mb-2">
                        HP Cutoff
                      </label>
                      <Select
                        value={config.accel_hp_cutoff?.toString() ?? "0"}
                        options={HP_CUTOFF_OPTIONS}
                        disabled={isDisabled}
                        onChange={(v) =>
                          updateConfig({
                            accel_hp_cutoff: toInt(v, config.accel_hp_cutoff ?? 0),
                          })
                        }
                      />
                    </div>
                  )}
                </div>
              </div>
            </section>

            {/* Gyroscope Settings */}
            <section>
              <div className="flex items-center justify-between mb-4">
                <h4 className="text-sm font-semibold text-muted uppercase tracking-wider">
                  Gyroscope
                </h4>
                <Switch
                  checked={config.gyro_enable ?? false}
                  disabled={isDisabled}
                  onChange={(checked) => updateConfig({ gyro_enable: !!checked })}
                  label=""
                />
              </div>

              <div className="grid grid-cols-2 lg:grid-cols-3 gap-4">
                <div>
                  <label className="text-xs text-muted block mb-2">Range</label>
                  <Select
                    value={config.gyro_range_dps?.toString() ?? "2000"}
                    options={GYRO_RANGE_OPTIONS}
                    disabled={isDisabled}
                    onChange={(v) =>
                      updateConfig({
                        gyro_range_dps: toInt(v, config.gyro_range_dps ?? 2000),
                      })
                    }
                  />
                </div>
                <div>
                  <label className="text-xs text-muted block mb-2">
                    Output Data Rate
                  </label>
                  <Select
                    value={findClosestOdr(config.gyro_odr_hz)}
                    options={ODR_OPTIONS}
                    disabled={isDisabled}
                    onChange={(v) =>
                      updateConfig({ gyro_odr_hz: toInt(v, config.gyro_odr_hz ?? 0) })
                    }
                  />
                </div>
              </div>

              {/* Advanced Filters */}
              <div className="mt-4 pt-4 border-t border-border/50">
                <p className="text-xs text-muted mb-3">Advanced Filters</p>
                <div className="grid grid-cols-2 lg:grid-cols-3 gap-4">
                  <div className="flex items-center justify-between bg-well rounded-lg px-4 py-3">
                    <span className="text-sm">LPF1 Enable</span>
                    <Switch
                      checked={config.gyro_lpf1_en ?? false}
                      disabled={isDisabled}
                      onChange={(checked) => updateConfig({ gyro_lpf1_en: !!checked })}
                      label=""
                    />
                  </div>
                  <div className="flex items-center justify-between bg-well rounded-lg px-4 py-3">
                    <span className="text-sm">HP Filter</span>
                    <Switch
                      checked={config.gyro_hp_en ?? false}
                      disabled={isDisabled}
                      onChange={(checked) => updateConfig({ gyro_hp_en: !!checked })}
                      label=""
                    />
                  </div>
                  <div className="flex items-center justify-between bg-well rounded-lg px-4 py-3">
                    <span className="text-sm">Low-Power/Normal Mode</span>
                    <Switch
                      checked={config.gyro_hm_mode ?? false}
                      disabled={isDisabled}
                      onChange={(checked) => updateConfig({ gyro_hm_mode: !!checked })}
                      label=""
                    />
                  </div>
                  {config.gyro_lpf1_en && (
                    <div>
                      <label className="text-xs text-muted block mb-2">
                        LPF1 Bandwidth
                      </label>
                      <Select
                        value={config.gyro_lpf1_bw?.toString() ?? "0"}
                        options={GYRO_LPF1_BW_OPTIONS}
                        disabled={isDisabled}
                        onChange={(v) =>
                          updateConfig({
                            gyro_lpf1_bw: toInt(v, config.gyro_lpf1_bw ?? 0),
                          })
                        }
                      />
                    </div>
                  )}
                  {config.gyro_hp_en && (
                    <div>
                      <label className="text-xs text-muted block mb-2">
                        HP Cutoff
                      </label>
                      <Select
                        value={config.gyro_hp_cutoff?.toString() ?? "0"}
                        options={HP_CUTOFF_OPTIONS}
                        disabled={isDisabled}
                        onChange={(v) =>
                          updateConfig({
                            gyro_hp_cutoff: toInt(v, config.gyro_hp_cutoff ?? 0),
                          })
                        }
                      />
                    </div>
                  )}
                </div>
              </div>
            </section>

            {/* System Info */}
            <section>
              <h4 className="text-sm font-semibold text-muted uppercase tracking-wider mb-4">
                System Information
              </h4>
              <div className="grid grid-cols-2 lg:grid-cols-4 gap-4">
                <div className="bg-well rounded-lg p-4">
                  <label className="text-xs text-muted block mb-1">
                    Chip ID
                  </label>
                  <p className="text-sm font-mono text-strong">
                    0x
                    {config.chip_id
                      ?.toString(16)
                      .toUpperCase()
                      .padStart(2, "0")}
                  </p>
                </div>
                <div className="bg-well rounded-lg p-4">
                  <label className="text-xs text-muted block mb-1">
                    Checksum
                  </label>
                  <p className="text-sm font-medium text-strong">
                    {config.checksum_algo === 1
                      ? "CRC8"
                      : config.checksum_algo === 2
                        ? "SUM8"
                        : config.checksum_algo === 3
                          ? "CRC8_HW"
                          : "Unknown"}
                  </p>
                </div>
                <div className="bg-well rounded-lg p-4 col-span-2">
                  <label className="text-xs text-muted block mb-1">
                    MAVLink Logging
                  </label>
                  <div className="flex items-center gap-2">
                    <span
                      className={`w-2 h-2 rounded-full ${config.mavlink_logging_enabled ? "bg-success" : "bg-muted"}`}
                    />
                    <p className="text-sm text-strong">
                      {config.mavlink_logging_enabled ? "Enabled" : "Disabled"}
                    </p>
                  </div>
                </div>
              </div>
            </section>

            {/* Save Button - Placeholder for future implementation */}
            <div className="flex justify-end pt-4 border-t border-border">
              <Button
                variant="filled"
                disabled={isDisabled || saving.value || !connected.value}
                onClick={handleSaveConfig}
              >
                {saving.value ? <Spinner size="s" /> : <Icon name="save" size="s" />}
                Save Configuration
              </Button>
            </div>
          </div>
        </div>
      )}

      {/* No Config State */}
      {!config?.config_valid && !configLoading.value && connected.value && (
        <div className="bg-panel border border-border rounded-xl p-12 flex flex-col items-center justify-center gap-4">
          <Icon name="settings" size="l" />
          <p className="text-muted text-center">
            No configuration received from logger.
            <br />
            The STM32 may still be initializing.
          </p>
          <Button variant="outlined" onClick={loadConfig}>
            <Icon name="refresh" size="s" />
            Retry
          </Button>
        </div>
      )}
    </div>
  );
};
