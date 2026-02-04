import styles from '../../progress-bar.module-D7CAXFAZ.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/progress-bar/progress-bar.tsx

// src/utils/cx/index.ts
function cx(...args) {
  let str = "";
  let i = 0;
  for (i = 0; i < args.length; i++) {
    const arg = args[i];
    if (arg) {
      if (typeof arg === "string") {
        str += (str ? " " : "") + arg;
      } else if (typeof arg === "object") {
        for (const key in arg) {
          if (arg[key]) {
            str += (str ? " " : "") + key;
          }
        }
      }
    }
  }
  return str;
}
function ProgressBar({ className, height = 8, value, max = 100, color, label, showValue = false }) {
  const progressBarStyle = {
    height: typeof height === "number" ? `${height}px` : height,
    boxShadow: "inset 0 2px 4px rgba(0, 0, 0, 0.1), inset 0 1px 2px rgba(0, 0, 0, 0.06)"
  };
  const percentage = value !== void 0 ? max === 0 ? 0 : Math.min(Math.max(value / max * 100, 0), 100) : void 0;
  const isIndeterminate = value === void 0;
  const progressValueStyle = {
    width: isIndeterminate ? "100%" : `${percentage}%`,
    backgroundColor: color,
    boxShadow: progressBarStyle.boxShadow
  };
  return /* @__PURE__ */ jsxs("div", { className: cx("w-full", className), children: [
    (label || showValue) && /* @__PURE__ */ jsxs("div", { className: "flex justify-between items-center mb-1", children: [
      label && /* @__PURE__ */ jsx("span", { className: "text-sm text-muted", children: label }),
      showValue && value !== void 0 && /* @__PURE__ */ jsxs("span", { className: "text-sm text-muted", children: [
        Math.round(percentage || 0),
        "%"
      ] })
    ] }),
    /* @__PURE__ */ jsx("div", { className: "w-full bg-primary/20 overflow-hidden rounded-full", style: progressBarStyle, children: /* @__PURE__ */ jsx(
      "div",
      {
        className: cx(styles["progress-bar-value"], {
          [styles["progress-bar-determinate"]]: !isIndeterminate
        }),
        style: progressValueStyle
      }
    ) })
  ] });
}

export { ProgressBar };
//# sourceMappingURL=progress-bar.js.map
//# sourceMappingURL=progress-bar.js.map