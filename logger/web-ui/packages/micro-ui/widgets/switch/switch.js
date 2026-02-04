import { useRef, useEffect } from 'preact/compat';
import styles from '../../switch.module-4KDNS6IS.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/switch/switch.tsx

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
var Switch = ({
  label,
  checked,
  disabled,
  id,
  onChange,
  className,
  onValueChange,
  valueSignal,
  treatBooleanAsNumber,
  onValidityChange,
  size = "m",
  loading = false
}) => {
  const lastValidityState = useRef(null);
  const handleChange = (e) => {
    if (loading) return;
    if (!(e.target instanceof HTMLInputElement)) return;
    const isChecked2 = e.target.checked;
    const outputValue = treatBooleanAsNumber ? isChecked2 ? 1 : 0 : isChecked2;
    callIf(onChange, outputValue);
    callIf(onValueChange, outputValue);
    setSignal(valueSignal, outputValue);
    if (onValidityChange) {
      const isValid = true;
      if (lastValidityState.current !== isValid) {
        lastValidityState.current = isValid;
        onValidityChange(isValid);
      }
    }
  };
  const isChecked = !!(valueSignal ? valueSignal.value : checked);
  useEffect(() => {
    const isValid = true;
    if (onValidityChange && lastValidityState.current !== isValid) {
      lastValidityState.current = isValid;
      onValidityChange(isValid);
    }
  }, []);
  const switchClasses = cx(styles.sw, className, styles[`s-${size}`], {
    [styles["sw--disabled"]]: disabled || loading,
    [styles["sw--loading"]]: loading
  });
  return /* @__PURE__ */ jsxs("label", { htmlFor: id, className: switchClasses, children: [
    /* @__PURE__ */ jsx("input", { id, type: "checkbox", className: styles.inp, checked: isChecked, disabled: disabled || loading, onChange: handleChange }),
    /* @__PURE__ */ jsxs("div", { className: styles.sc, children: [
      /* @__PURE__ */ jsx("div", { className: styles.t }),
      /* @__PURE__ */ jsx("div", { className: styles.th })
    ] }),
    label && /* @__PURE__ */ jsx("span", { className: styles.lbl, children: label })
  ] });
};

export { Switch };
//# sourceMappingURL=switch.js.map
//# sourceMappingURL=switch.js.map