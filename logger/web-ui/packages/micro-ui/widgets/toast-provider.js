import { useEffect as useEffect$1 } from 'preact/hooks';
import { useSignal } from '@preact/signals';
import * as toast_service_star from 'micro-ui/widgets/toast-service';
import { toastService } from 'micro-ui/widgets/toast-service';
import { useEffect } from 'preact/compat';
import styles from '../icon.module-K7UO6GPK.module.css';
import { jsx, jsxs, Fragment } from 'preact/jsx-runtime';
import styles2 from '../toast.module-HXXQU2AL.module.css';

var __defProp = Object.defineProperty;
var __getOwnPropDesc = Object.getOwnPropertyDescriptor;
var __getOwnPropNames = Object.getOwnPropertyNames;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __export = (target, all) => {
  for (var name in all)
    __defProp(target, name, { get: all[name], enumerable: true });
};
var __copyProps = (to, from, except, desc) => {
  if (from && typeof from === "object" || typeof from === "function") {
    for (let key of __getOwnPropNames(from))
      if (!__hasOwnProp.call(to, key) && key !== except)
        __defProp(to, key, { get: () => from[key], enumerable: !(desc = __getOwnPropDesc(from, key)) || desc.enumerable });
  }
  return to;
};
var __reExport = (target, mod, secondTarget) => (__copyProps(target, mod, "default"), secondTarget);

// src/widgets/toast/index.ts
var toast_exports = {};
__export(toast_exports, {
  Toast: () => Toast,
  ToastProvider: () => ToastProvider
});

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
var ToastProvider = ({ children }) => {
  const toasts = useSignal([]);
  const showToast = (options) => {
    const id = crypto.randomUUID();
    const newToast = {
      duration: 4e3,
      position: "top-center",
      type: "info",
      ...options,
      id
    };
    toasts.value = [...toasts.value, newToast];
    if (newToast.duration !== null) {
      setTimeout(() => hideToast(id), newToast.duration);
    }
    return id;
  };
  const hideToast = (id) => {
    toasts.value = toasts.value.filter((toast) => toast.id !== id);
  };
  useEffect$1(() => {
    toastService.register(showToast, hideToast);
  }, []);
  const topToasts = toasts.value.filter((t) => t.position === "top-center");
  const bottomToasts = toasts.value.filter((t) => t.position === "bottom-center");
  return /* @__PURE__ */ jsxs(Fragment, { children: [
    children,
    /* @__PURE__ */ jsx("div", { className: "fixed top-4 left-1/2 -translate-x-1/2 z-[9999] w-full max-w-sm flex flex-col items-center pointer-events-none", children: topToasts.map((toast) => /* @__PURE__ */ jsx(Toast, { options: toast, onClose: () => hideToast(toast.id) }, toast.id)) }),
    /* @__PURE__ */ jsx("div", { className: "fixed bottom-4 left-1/2 -translate-x-1/2 z-[9999] w-full max-w-sm flex flex-col-reverse items-center pointer-events-none", children: bottomToasts.map((toast) => /* @__PURE__ */ jsx(Toast, { options: toast, onClose: () => hideToast(toast.id) }, toast.id)) })
  ] });
};

// src/widgets/toast/index.ts
__reExport(toast_exports, toast_service_star);
var ToastProvider2 = ({ children }) => {
  const toasts = useSignal([]);
  const showToast = (options) => {
    const id = `toast-${Date.now()}-${Math.random()}`;
    const newToast = {
      duration: 4e3,
      position: "top-center",
      type: "info",
      ...options,
      id
    };
    toasts.value = [...toasts.value, newToast];
    if (newToast.duration) {
      setTimeout(() => hideToast(id), newToast.duration);
    }
    return id;
  };
  const hideToast = (id) => {
    toasts.value = toasts.value.filter((toast) => toast.id !== id);
  };
  useEffect$1(() => {
    toastService.register(showToast, hideToast);
  }, []);
  const topToasts = toasts.value.filter((t) => t.position === "top-center");
  const bottomToasts = toasts.value.filter((t) => t.position === "bottom-center");
  return /* @__PURE__ */ jsxs(Fragment, { children: [
    children,
    /* @__PURE__ */ jsx("div", { className: "fixed top-4 left-1/2 -translate-x-1/2 z-[9999] w-full max-w-sm flex flex-col items-center pointer-events-none", children: topToasts.map((toast) => /* @__PURE__ */ jsx(Toast, { options: toast, onClose: () => hideToast(toast.id) }, toast.id)) }),
    /* @__PURE__ */ jsx("div", { className: "fixed bottom-4 left-1/2 -translate-x-1/2 z-[9999] w-full max-w-sm flex flex-col-reverse items-center pointer-events-none", children: bottomToasts.map((toast) => /* @__PURE__ */ jsx(Toast, { options: toast, onClose: () => hideToast(toast.id) }, toast.id)) })
  ] });
};

export { ToastProvider2 as ToastProvider };
//# sourceMappingURL=toast-provider.js.map
//# sourceMappingURL=toast-provider.js.map