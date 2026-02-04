import { useEffect } from 'preact/compat';
import { useSignal } from '@preact/signals';
import styles from '../../icon.module-K7UO6GPK.module.css';
import { jsx, jsxs } from 'preact/jsx-runtime';
import styles2 from '../../toast.module-HXXQU2AL.module.css';

// src/widgets/toast/toast.tsx

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
var getIconName = (type) => {
  switch (type) {
    case "error":
      return "warning";
    case "warning":
      return "warning_amber";
    case "success":
      return "check_circle";
    case "info":
    default:
      return "info";
  }
};
var typeClasses = {
  info: "text-info",
  success: "text-success",
  warning: "text-warn",
  error: "text-error"
};
var Toast = ({ options, onClose }) => {
  const { id, message, title, type = "info", icon, component: CustomComponent, props: customProps, onClick, duration } = options;
  const isVisible = useSignal(false);
  useEffect(() => {
    const timer = setTimeout(() => isVisible.value = true, 50);
    return () => clearTimeout(timer);
  }, []);
  const handleClose = () => {
    isVisible.value = false;
    setTimeout(onClose, 400);
  };
  const handleClick = () => {
    if (onClick) {
      onClick();
    }
    if (duration !== null) {
      handleClose();
    }
  };
  const shadowClass = `shadow-toast-${type}`;
  const isTopPosition = options.position?.startsWith("top");
  const transformClass = isTopPosition ? "-translate-y-full" : "translate-y-full";
  return /* @__PURE__ */ jsx(
    "div",
    {
      className: cx(
        styles2.t,
        styles2[type === "info" ? "i" : type === "success" ? "s" : type === "warning" ? "w" : "e"],
        "relative flex items-center w-full max-w-sm p-3 my-2 rounded-lg overflow-hidden cursor-pointer pointer-events-auto shadow-md",
        "bg-overlay",
        typeClasses[type],
        shadowClass,
        "transition-all duration-300 ease-in-out",
        isVisible.value ? "opacity-100 translate-y-0" : `opacity-0 ${transformClass}`
      ),
      onClick: CustomComponent ? void 0 : handleClick,
      children: CustomComponent ? /* @__PURE__ */ jsx(CustomComponent, { ...customProps, toastId: id, closeToast: handleClose }) : /* @__PURE__ */ jsxs("div", { className: "flex items-start w-full", children: [
        /* @__PURE__ */ jsx("div", { className: "shrink-0 pt-1", children: /* @__PURE__ */ jsx(Icon, { name: icon || getIconName(type), size: "m", type }) }),
        /* @__PURE__ */ jsxs("div", { className: "ml-3 flex-1", children: [
          title && /* @__PURE__ */ jsx("p", { className: "text-base font-bold text-inv", children: title }),
          /* @__PURE__ */ jsx("p", { className: "text-sm", children: message })
        ] }),
        duration === null && /* @__PURE__ */ jsx("div", { className: "shrink-0 ml-4", children: /* @__PURE__ */ jsx("button", { onClick: handleClose, className: "p-1 -m-1 rounded-full hover:bg-secondary/30 transition-colors", children: /* @__PURE__ */ jsx(Icon, { name: "close", size: "s" }) }) })
      ] })
    }
  );
};

export { Toast };
//# sourceMappingURL=toast.js.map
//# sourceMappingURL=toast.js.map