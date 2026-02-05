import { jsxs, jsx } from 'preact/jsx-runtime';

// src/widgets/select/select-modal-list.tsx
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

export { SelectModalList };
//# sourceMappingURL=select-modal-list.js.map
//# sourceMappingURL=select-modal-list.js.map