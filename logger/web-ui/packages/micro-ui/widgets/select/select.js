import { useState, useRef, useEffect } from 'preact/hooks';
import { jsxs, jsx } from 'preact/jsx-runtime';
import styles from '../../icon.module-K7UO6GPK.module.css';
import { modalService } from 'micro-ui/widgets/modal-service';
import { mobileViewService } from 'micro-ui/widgets/mobile-view-service';
import styles2 from '../../select.module-G7LKCEII.module.css';

// src/widgets/select/select.tsx
var SelectModalList = ({ options, currentValue, onSelect, title }) => {
  const getOptionParts = (option) => {
    if (typeof option === "object" && option !== null && "name" in option && "value" in option) {
      return { name: option.name, value: option.value };
    }
    return { name: String(option), value: option };
  };
  return /* @__PURE__ */ jsxs("div", { className: "flex flex-col p-2 bg-base rounded-lg shadow-lg", children: [
    /* @__PURE__ */ jsx("h2", { className: "text-xl font-semibold mb-4 text-center", children: title || "Select Option" }),
    /* @__PURE__ */ jsx("div", { className: "max-h-[60dvh] overflow-y-auto -mr-4 pr-4", children: /* @__PURE__ */ jsx("ul", { className: "flex flex-col space-y-1 ", children: options.map((option) => {
      const { name, value } = getOptionParts(option);
      const isActive = value === currentValue;
      return /* @__PURE__ */ jsx("li", { className: "w-full shadow-md", children: /* @__PURE__ */ jsx(
        "button",
        {
          type: "button",
          onClick: () => onSelect(value),
          className: `w-full text-left p-3 rounded-md transition-colors cursor-pointer ${isActive ? "bg-primary/20 text-primary" : "hover:bg-overlay"}`,
          children: name
        }
      ) }, value);
    }) }) })
  ] });
};

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
var Icon = (props) => {
  const typeClass = props.type ? styles[`${props.type}-icon`] : null;
  const classes = cx(styles.ic, styles[`s-${props.size || "m"}`], typeClass, props.classes);
  const wrapperClasses = cx(styles.wrapper, {
    [styles.iconDisabled]: props.isDisabled
  });
  return /* @__PURE__ */ jsxs("div", { className: wrapperClasses, onClick: props.onClick, style: props.style, children: [
    /* @__PURE__ */ jsx("svg", { className: classes, children: /* @__PURE__ */ jsx("use", { xlinkHref: `#${props.name}` }) }),
    props.subscript && /* @__PURE__ */ jsx("span", { className: cx(styles.badge), children: props.subscript })
  ] });
};

// src/utils/callbacks.ts
var callIf = (fn, ...args) => {
  if (fn) fn(...args);
};
var setSignal = (signal, value) => {
  if (signal) signal.value = value;
};
var DEFAULT_PLACEHOLDER = "widgets.select.placeholder";
var Select = ({
  options,
  value,
  onChange,
  placeholder = DEFAULT_PLACEHOLDER,
  disabled,
  className,
  valueSignal,
  title,
  onValueChange,
  onValidityChange,
  required
}) => {
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [dropdownDirection, setDropdownDirection] = useState("bottom");
  const selRef = useRef(null);
  const getOptionParts = (option) => {
    if (typeof option === "object" && option !== null && "name" in option && "value" in option) {
      return { name: option.name, value: option.value };
    }
    return { name: String(option), value: option };
  };
  const selectedValue = valueSignal ? valueSignal.value : value;
  const handleSelect = (newValue) => {
    callIf(onChange, newValue);
    callIf(onValueChange, newValue);
    setSignal(valueSignal, newValue);
    setIsDropdownOpen(false);
  };
  const handleTriggerClick = () => {
    if (disabled) return;
    if (mobileViewService.isMobile.value) {
      modalService.show(SelectModalList, {
        options,
        title,
        currentValue: selectedValue,
        onSelect: (newValue) => {
          handleSelect(newValue);
          modalService.hide();
        }
      });
    } else {
      if (!isDropdownOpen && selRef.current) {
        const rect = selRef.current.getBoundingClientRect();
        const spaceBelow = window.innerHeight - rect.bottom;
        const dropdownHeight = 250;
        setDropdownDirection(spaceBelow < dropdownHeight && rect.top > dropdownHeight ? "top" : "bottom");
      }
      setIsDropdownOpen(!isDropdownOpen);
    }
  };
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (selRef.current && !selRef.current.contains(event.target)) {
        setIsDropdownOpen(false);
      }
    };
    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, [selRef]);
  const lastValidityState = useRef(null);
  useEffect(() => {
    const currentValue = valueSignal ? valueSignal.value : value;
    const isValid = !required || currentValue !== void 0;
    if (onValidityChange && lastValidityState.current !== isValid) {
      lastValidityState.current = isValid;
      onValidityChange(isValid);
    }
  }, [required, value, valueSignal?.value, onValidityChange]);
  const currentOption = options.find((opt) => getOptionParts(opt).value === selectedValue);
  const displayValue = currentOption ? getOptionParts(currentOption).name : placeholder ?? DEFAULT_PLACEHOLDER;
  return /* @__PURE__ */ jsxs("div", { ref: selRef, className: cx("relative w-full", className), children: [
    /* @__PURE__ */ jsxs(
      "button",
      {
        type: "button",
        onClick: handleTriggerClick,
        disabled,
        className: cx(styles2.trigger, "flex items-center justify-between w-full px-2 py-1 text-left cursor-pointer"),
        children: [
          /* @__PURE__ */ jsx("span", { children: displayValue }),
          /* @__PURE__ */ jsx(Icon, { name: "arrow_right", classes: cx("transition-transform duration-200", isDropdownOpen ? "rotate-[270deg]" : "rotate-90") })
        ]
      }
    ),
    isDropdownOpen && !mobileViewService.isMobile.value && /* @__PURE__ */ jsx(
      "div",
      {
        className: cx(
          "absolute left-0 right-0 z-50 bg-panel border border-border rounded-md shadow-lg",
          dropdownDirection === "bottom" ? "top-full mt-1" : "bottom-full mb-1"
        ),
        children: /* @__PURE__ */ jsx("ul", { className: "flex flex-col gap-1 p-1 overflow-y-auto max-h-60", children: options.map((option) => {
          const { name: optionName, value: optionValue } = getOptionParts(option);
          const isActive = optionValue === selectedValue;
          return /* @__PURE__ */ jsx("li", { children: /* @__PURE__ */ jsx(
            "button",
            {
              type: "button",
              onClick: () => handleSelect(optionValue),
              className: cx(
                "w-full p-2 text-left rounded cursor-pointer transition-colors text-inv",
                isActive ? "bg-primary/30" : "hover:bg-overlay"
              ),
              children: optionName
            }
          ) }, optionValue);
        }) })
      }
    )
  ] });
};

export { Select };
//# sourceMappingURL=select.js.map
//# sourceMappingURL=select.js.map