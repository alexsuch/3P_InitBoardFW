import styles from '../../checkbox.module-JB5MX5CY.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/checkbox/checkbox.tsx

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
var Checkbox = ({
  label,
  checked,
  disabled,
  id,
  onChange,
  className,
  onValueChange,
  valueSignal,
  size = "m",
  treatBooleanAsNumber
}) => {
  const handleChange = (e) => {
    if (!(e.target instanceof HTMLInputElement)) return;
    const isChecked2 = e.target.checked;
    const outputValue = treatBooleanAsNumber ? isChecked2 ? 1 : 0 : isChecked2;
    setSignal(valueSignal, outputValue);
    callIf(onChange, outputValue);
    callIf(onValueChange, outputValue);
  };
  const isChecked = !!(valueSignal ? valueSignal.value : checked);
  const checkboxClasses = cx(styles.cb, styles[`s-${size}`], className, { [styles["cb--disabled"]]: disabled });
  return /* @__PURE__ */ jsxs("label", { htmlFor: id, className: checkboxClasses, children: [
    /* @__PURE__ */ jsx("input", { id, type: "checkbox", className: styles.inp, checked: isChecked, disabled, onChange: handleChange }),
    /* @__PURE__ */ jsx("div", { className: styles.box, children: /* @__PURE__ */ jsx("svg", { className: styles.cm, viewBox: "0 0 24 24", fill: "none", xmlns: "http://www.w3.org/2000/svg", children: /* @__PURE__ */ jsx("polyline", { points: "20 6 9 17 4 12" }) }) }),
    label && /* @__PURE__ */ jsx("span", { className: styles.lbl, children: label })
  ] });
};

export { Checkbox };
//# sourceMappingURL=checkbox.js.map
//# sourceMappingURL=checkbox.js.map