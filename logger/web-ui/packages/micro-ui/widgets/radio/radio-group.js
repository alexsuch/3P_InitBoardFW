import styles from '../../radio.module-UGJD6HST.module.css';
import { useEffect } from 'preact/hooks';
import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/radio/radio-group.tsx

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
var RadioGroup = ({
  label,
  options,
  value,
  valueSignal,
  name,
  disabled,
  className,
  onValueChange,
  onValidityChange,
  layout = "vertical"
}) => {
  const getOptionParts = (option) => {
    if (typeof option === "object" && option !== null && "name" in option && "value" in option) {
      return { name: option.name, value: option.value };
    }
    return { name: String(option), value: option };
  };
  const handleChange = (e) => {
    if (!(e.target instanceof HTMLInputElement)) return;
    const newValue = e.target.value;
    const selectedOption = options.map(getOptionParts).find((opt) => String(opt.value) === newValue);
    const outputValue = selectedOption ? selectedOption.value : newValue;
    setSignal(valueSignal, outputValue);
    callIf(onValueChange, outputValue);
    callIf(onValidityChange, true);
  };
  useEffect(() => {
    callIf(onValidityChange, true);
  }, [onValidityChange]);
  const currentValue = valueSignal ? valueSignal.value : value;
  const groupClasses = cx(styles.rg, className, layout === "horizontal" ? styles.gh : styles.gv);
  return /* @__PURE__ */ jsxs("div", { className: groupClasses, role: "radiogroup", "aria-label": label, children: [
    label && /* @__PURE__ */ jsx("span", { className: cx(styles.gl, { [styles.gld]: disabled }), children: label }),
    /* @__PURE__ */ jsx("div", { className: styles.oc, children: options.map((option) => {
      const { name: optionName, value: optionValue } = getOptionParts(option);
      const isChecked = currentValue === optionValue;
      const optionId = `radio-${name}-${optionValue}`;
      return /* @__PURE__ */ jsxs("label", { htmlFor: optionId, className: cx(styles.r, { [styles.rd]: disabled }), children: [
        /* @__PURE__ */ jsx(
          "input",
          {
            id: optionId,
            type: "radio",
            className: styles.inp,
            name,
            value: optionValue,
            checked: isChecked,
            disabled,
            onChange: handleChange
          }
        ),
        /* @__PURE__ */ jsx("span", { className: styles.ctrl, children: /* @__PURE__ */ jsx("span", { className: styles.dot }) }),
        /* @__PURE__ */ jsx("span", { className: styles.lbl, children: optionName })
      ] }, optionValue);
    }) })
  ] });
};

export { RadioGroup };
//# sourceMappingURL=radio-group.js.map
//# sourceMappingURL=radio-group.js.map