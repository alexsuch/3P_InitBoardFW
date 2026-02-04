import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/alert/alert.tsx
var alertStyles = {
  info: {
    container: "bg-info/10 border-info/30 text-info",
    icon: "\u2139\uFE0F"
  },
  success: {
    container: "bg-success/10 border-success/30 text-success",
    icon: "\u2705"
  },
  warning: {
    container: "bg-warn/10 border-warn/30 text-warn",
    icon: "\u26A0\uFE0F"
  },
  error: {
    container: "bg-error/10 border-error/30 text-error",
    icon: "\u274C"
  }
};
var Alert = ({ type = "info", title, children, onRetry, className = "", retryLabel = "Retry" }) => {
  const styles = alertStyles[type];
  return /* @__PURE__ */ jsxs("div", { class: `flex flex-col p-4 space-y-4 ${className}`, children: [
    /* @__PURE__ */ jsx("div", { class: `border px-4 py-3 rounded ${styles.container}`, children: /* @__PURE__ */ jsxs("div", { class: "flex items-start", children: [
      /* @__PURE__ */ jsx("span", { class: "mr-2 text-lg", children: styles.icon }),
      /* @__PURE__ */ jsxs("div", { class: "flex-1", children: [
        title && /* @__PURE__ */ jsx("strong", { class: "font-bold block", children: title }),
        /* @__PURE__ */ jsx("span", { class: "block sm:inline mt-1", children })
      ] })
    ] }) }),
    onRetry && /* @__PURE__ */ jsx(
      "button",
      {
        onClick: onRetry,
        class: "bg-primary hover:bg-primary-hover text-inv font-bold py-2 px-4 rounded transition-colors duration-200",
        children: retryLabel
      }
    )
  ] });
};

export { Alert };
//# sourceMappingURL=alert.js.map
//# sourceMappingURL=alert.js.map