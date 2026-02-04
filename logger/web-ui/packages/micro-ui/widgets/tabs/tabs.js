import { useSignal } from '@preact/signals';
import styles from '../../tabs.module-K26DHJHW.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/tabs/tabs.tsx

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

// src/utils/callbacks.ts
var callIf = (fn, ...args) => {
  if (fn) fn(...args);
};
var setSignal = (signal, value) => {
  if (signal) signal.value = value;
};
var Tabs = ({ items, initialValue, value, onValueChange, valueSignal, className, panelClassName }) => {
  const isControlled = valueSignal !== void 0 || value !== void 0;
  const localSignal = useSignal(initialValue || items[0]?.value || "");
  const activeValue = isControlled ? valueSignal?.value || value || "" : localSignal.value || "";
  const handleTabClick = (newValue) => {
    if (!isControlled) {
      localSignal.value = newValue;
    }
    callIf(onValueChange, newValue);
    setSignal(valueSignal, newValue);
  };
  const activeContent = items.find((item) => item.value === activeValue)?.content;
  return /* @__PURE__ */ jsxs("div", { className, children: [
    /* @__PURE__ */ jsx("div", { className: styles.tl, role: "tablist", children: items.map((item) => {
      const isActive = item.value === activeValue;
      return /* @__PURE__ */ jsx(
        "button",
        {
          role: "tab",
          type: "button",
          disabled: item.disabled,
          "aria-selected": isActive,
          className: cx(styles.tab, { [styles.active]: isActive }),
          onClick: () => !item.disabled && handleTabClick(item.value),
          children: item.label
        },
        item.value
      );
    }) }),
    /* @__PURE__ */ jsx("div", { className: cx(styles.tp, panelClassName), role: "tabpanel", children: activeContent })
  ] });
};

export { Tabs };
//# sourceMappingURL=tabs.js.map
//# sourceMappingURL=tabs.js.map