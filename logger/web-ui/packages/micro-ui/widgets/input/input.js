import { useSignal } from '@preact/signals';
import { forwardRef, useRef, useEffect } from 'preact/compat';
import styles from '../../input.module-DD44CNJ5.module.css';
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
var Input = forwardRef(
  ({
    name,
    label,
    placeholder,
    className,
    id,
    type,
    initialValue,
    onChange,
    value,
    valueSignal,
    onValueChange,
    min,
    max,
    error,
    hint,
    validators,
    disabled,
    inputTextClasses,
    allowedCharsPattern,
    onValidityChange,
    pasteConverter,
    ...rest
  }, ref) => {
    const errorSignal = useSignal(error);
    const hintSignal = useSignal(hint);
    const lastValidityState = useRef(null);
    const validateValue = (currentValue) => {
      let hasError = false;
      const validationMessages = [];
      if (validators && validators.length > 0) {
        validators.forEach((validator) => {
          const vr = validator.validate(currentValue);
          if (vr?.error) {
            hasError = true;
            if (vr.message) validationMessages.push(vr.message);
          }
        });
      }
      if (allowedCharsPattern && String(currentValue).length > 0) {
        const flags = allowedCharsPattern.flags.replace("g", "").replace("y", "");
        const singleCharRe = new RegExp(`^(?:${allowedCharsPattern.source})$`, flags);
        const str = String(currentValue);
        const hasInvalid = Array.from(str).some((ch) => !singleCharRe.test(ch));
        if (hasInvalid) {
          hasError = true;
          validationMessages.push("Invalid characters entered.");
        }
      }
      return { hasError, validationMessages };
    };
    useEffect(() => {
      const currentValue = (valueSignal ? valueSignal.value : value) ?? initialValue ?? "";
      const { hasError, validationMessages } = validateValue(currentValue);
      const isValid = !hasError;
      if (error && String(error).length > 0) {
        errorSignal.value = error;
      } else {
        errorSignal.value = hasError ? validationMessages.join("; ") : "";
      }
      hintSignal.value = hint;
      if (onValidityChange && lastValidityState.current !== isValid) {
        lastValidityState.current = isValid;
        onValidityChange(isValid);
      }
    }, [value, valueSignal?.value, validators, allowedCharsPattern, onValidityChange, error, hint]);
    const handleKeyDown = (e) => {
      if (!allowedCharsPattern) return;
      const flags = allowedCharsPattern.flags.replace("g", "").replace("y", "");
      const singleCharRe = new RegExp(`^(?:${allowedCharsPattern.source})$`, flags);
      if (["Backspace", "Delete", "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "Tab", "Enter", "Home", "End", "Escape"].includes(
        e.key
      ) || e.ctrlKey || e.metaKey) {
        return;
      }
      if (!singleCharRe.test(e.key)) {
        e.preventDefault();
      }
    };
    const handleInput = (e) => {
      if (!(e.target instanceof HTMLInputElement)) return;
      const currentValue = e.target.value;
      const { hasError, validationMessages } = validateValue(currentValue);
      errorSignal.value = hasError ? validationMessages.join("; ") : "";
      const isValid = !hasError;
      if (onValidityChange && lastValidityState.current !== isValid) {
        lastValidityState.current = isValid;
        onValidityChange(isValid);
      }
      callIf(onChange, e);
      callIf(onValueChange, currentValue);
      setSignal(valueSignal, currentValue);
    };
    const handlePaste = async (e) => {
      if (!pasteConverter) return;
      e.preventDefault();
      const pastedData = e.clipboardData?.getData("text/plain") || "";
      try {
        const convertedValue = pasteConverter(pastedData);
        setSignal(valueSignal, convertedValue);
        callIf(onValueChange, convertedValue);
        callIf(onChange, { target: { value: convertedValue } });
        const { hasError, validationMessages } = validateValue(convertedValue);
        errorSignal.value = hasError ? validationMessages.join("; ") : "";
        const isValid = !hasError;
        if (onValidityChange && lastValidityState.current !== isValid) {
          lastValidityState.current = isValid;
          onValidityChange(isValid);
        }
      } catch (error2) {
        if (typeof import.meta !== "undefined" && import.meta.env && import.meta.env.DEV) {
          console.warn("Paste converter error:", error2);
        }
        setSignal(valueSignal, pastedData);
        callIf(onValueChange, pastedData);
        callIf(onChange, { target: { value: pastedData } });
        const { hasError, validationMessages } = validateValue(pastedData);
        errorSignal.value = hasError ? validationMessages.join("; ") : "";
        const isValid = !hasError;
        if (onValidityChange && lastValidityState.current !== isValid) {
          lastValidityState.current = isValid;
          onValidityChange(isValid);
        }
      }
    };
    const displayValue = (valueSignal ? valueSignal.value : value) ?? initialValue;
    return /* @__PURE__ */ jsxs("div", { className: cx("flex flex-col", className), children: [
      label && /* @__PURE__ */ jsx("label", { htmlFor: id, className: cx(styles.l, { [styles.ld]: disabled }), children: label }),
      /* @__PURE__ */ jsx("div", { className: "flex w-full", children: /* @__PURE__ */ jsx(
        "input",
        {
          id,
          ref,
          type,
          name,
          placeholder,
          min,
          max,
          value: displayValue,
          onInput: handleInput,
          onKeyDown: handleKeyDown,
          onPaste: handlePaste,
          disabled,
          className: cx(styles.i, "w-full flex-1", inputTextClasses, {
            [styles.ie]: !!errorSignal.value
          }),
          ...rest
        }
      ) }),
      hintSignal.value && !errorSignal.value && /* @__PURE__ */ jsx("span", { className: "mt-1 text-xs text-muted", children: hintSignal.value }),
      errorSignal.value && /* @__PURE__ */ jsx("span", { className: "mt-1 text-xs text-error", children: errorSignal.value })
    ] });
  }
);

export { Input };
//# sourceMappingURL=input.js.map
//# sourceMappingURL=input.js.map