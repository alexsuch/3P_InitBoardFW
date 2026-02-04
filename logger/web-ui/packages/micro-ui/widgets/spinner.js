import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/spinner/spinner.tsx
var getSpinnerSize = (size) => {
  switch (size) {
    case "xs":
      return "h-4 w-4";
    case "s":
      return "h-6 w-6";
    case "m":
      return "h-8 w-8";
    case "l":
      return "h-12 w-12";
    case "xl":
      return "h-16 w-16";
    default:
      return "h-8 w-8";
  }
};
var getTextSize = (size) => {
  switch (size) {
    case "xs":
      return "text-xs";
    case "s":
      return "text-sm";
    case "m":
      return "text-base";
    case "l":
      return "text-lg";
    case "xl":
      return "text-xl";
    default:
      return "text-base";
  }
};
var Spinner = ({ size = "m", text, className = "", textClassName = "" }) => {
  const spinnerSize = getSpinnerSize(size);
  const textSize = getTextSize(size);
  return /* @__PURE__ */ jsxs("div", { class: `flex flex-col items-center justify-center ${className}`, children: [
    /* @__PURE__ */ jsx("div", { class: `animate-spin rounded-full border-b-2 border-primary ${spinnerSize}` }),
    text && /* @__PURE__ */ jsx("p", { class: `mt-2 text-muted ${textSize} ${textClassName}`, children: text })
  ] });
};

export { Spinner };
//# sourceMappingURL=spinner.js.map
//# sourceMappingURL=spinner.js.map