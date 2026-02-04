import { useSignal } from '@preact/signals';
import { forwardRef, useRef, useEffect } from 'preact/compat';
import styles from '../../slider.module-2PFEQ466.module.css';
import { jsxs, jsx } from 'preact/jsx-runtime';

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
var Slider = forwardRef(
  ({ id, label, className, min = 0, max = 100, step = 1, value, valueSignal, onValueChange, disabled, showValue = true }, ref) => {
    const localVal = useSignal(valueSignal ? valueSignal.value : value ?? min);
    const sliderRef = useRef(null);
    const displayValue = valueSignal ? valueSignal.value : value ?? localVal.value;
    useEffect(() => {
      if (sliderRef.current) {
        const numericMin = Number(min);
        const numericMax = Number(max);
        const numericValue = Number(displayValue);
        const safeMin = Number.isFinite(numericMin) ? numericMin : 0;
        const safeMax = Number.isFinite(numericMax) ? numericMax : safeMin;
        const lowerBound = Math.min(safeMin, safeMax);
        const upperBound = Math.max(safeMin, safeMax);
        const finiteValue = Number.isFinite(numericValue) ? numericValue : lowerBound;
        const clampedValue = Math.min(Math.max(finiteValue, lowerBound), upperBound);
        const denominator = upperBound - lowerBound;
        const rawPercentage = denominator === 0 ? 0 : (clampedValue - lowerBound) / denominator * 100;
        const percentage = Math.max(0, Math.min(100, rawPercentage));
        sliderRef.current.style.setProperty("--track-fill", `${percentage}%`);
      }
    }, [displayValue, min, max]);
    const handleInput = (e) => {
      const v = Number(e.currentTarget.value);
      localVal.value = v;
      setSignal(valueSignal, v);
      callIf(onValueChange, v);
    };
    return /* @__PURE__ */ jsxs("div", { className: cx(className, "flex flex-col"), children: [
      label && /* @__PURE__ */ jsx("div", { className: cx(styles.label, "flex", { [styles["label-disabled"]]: disabled }), children: /* @__PURE__ */ jsx("span", { children: label }) }),
      /* @__PURE__ */ jsxs("div", { className: "flex items-center space-x-2", children: [
        /* @__PURE__ */ jsx(
          "input",
          {
            ref: (el) => {
              sliderRef.current = el;
              if (typeof ref === "function") {
                ref(el);
              } else if (ref) {
                ref.current = el;
              }
            },
            id,
            type: "range",
            min,
            max,
            step,
            value: displayValue,
            onInput: handleInput,
            disabled,
            className: cx(styles.slider, "flex-1")
          }
        ),
        showValue && /* @__PURE__ */ jsx("span", { className: cx("text-xs w-8 text-right", styles.value), children: displayValue })
      ] })
    ] });
  }
);

export { Slider };
//# sourceMappingURL=slider.js.map
//# sourceMappingURL=slider.js.map