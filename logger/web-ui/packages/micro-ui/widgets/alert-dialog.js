import { createContext } from 'preact';
import { useEffect, useContext, useRef } from 'preact/hooks';
import styles2 from '../button.module-APT35VFH.module.css';
import styles from '../icon.module-K7UO6GPK.module.css';
import { jsxs, Fragment, jsx } from 'preact/jsx-runtime';
import { modalService } from 'micro-ui/widgets/modal-service';
import { alertService } from 'micro-ui/widgets/alert-service';

// src/widgets/alert-dialog/alert-dialog.tsx

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
var Button = ({
  variant = "elevated",
  color = "primary",
  iconOnly = false,
  iconAlignment = "left",
  size = "m",
  iconName,
  iconSize,
  iconClassName,
  className,
  children,
  text,
  onClick,
  onMouseDown,
  onMouseUp,
  onPointerDown,
  id,
  disabled,
  isDisabled,
  ...rest
}) => {
  const btnRef = useRef(null);
  const isDisabledState = disabled ?? isDisabled;
  const createRipple = (e) => {
    const button = btnRef.current;
    if (!button || isDisabledState) return;
    const ripple = document.createElement("span");
    const rect = button.getBoundingClientRect();
    const size2 = Math.max(rect.width, rect.height);
    const x = e.clientX - rect.left - size2 / 2;
    const y = e.clientY - rect.top - size2 / 2;
    ripple.style.width = ripple.style.height = `${size2}px`;
    ripple.style.left = `${x}px`;
    ripple.style.top = `${y}px`;
    ripple.className = "ripple-base";
    button.appendChild(ripple);
    setTimeout(() => {
      ripple.remove();
    }, 600);
  };
  const handleMouseDown = (e) => {
    createRipple(e);
    if (onMouseDown) {
      onMouseDown(e);
    }
  };
  const handlePointerDown = (e) => {
    createRipple(e);
    if (onPointerDown) {
      onPointerDown(e);
    }
  };
  const isIconOnly = iconOnly || iconName && !text && !children;
  const showText = !isIconOnly && (text || children);
  const alignmentClass = isIconOnly || !iconName ? "justify-center text-center" : iconAlignment === "left" ? "justify-start text-left" : iconAlignment === "right" ? "justify-end text-right" : "justify-center text-center";
  const colorMap = {
    primary: "cp",
    secondary: "cs",
    warning: "cw",
    success: "csu",
    error: "ce",
    inherit: "ci"
  };
  const variantMap = {
    filled: "vf",
    elevated: "ve",
    outlined: "vo",
    text: "vt",
    ghost: "vg"
  };
  const sizeMap = {
    xs: "s-xs",
    s: "s-s",
    m: "s-m",
    l: "s-l",
    xl: "s-xl"
  };
  const classnames = cx(
    className,
    styles2.b,
    styles2[colorMap[color]],
    styles2[variantMap[variant]],
    styles2[sizeMap[size]],
    { [styles2.io]: isIconOnly },
    "flex items-center",
    alignmentClass
  );
  const iconEl = iconName && /* @__PURE__ */ jsx(
    Icon,
    {
      name: iconName,
      isDisabled,
      size: iconSize,
      classes: cx(iconClassName, {
        "mr-2": showText && iconAlignment !== "right",
        "ml-2": showText && iconAlignment === "right"
      })
    }
  );
  return /* @__PURE__ */ jsxs(
    "button",
    {
      ref: btnRef,
      id,
      "data-testid": id,
      className: classnames,
      onClick,
      onMouseDown: handleMouseDown,
      onMouseUp,
      onPointerDown: handlePointerDown,
      disabled: isDisabledState,
      ...rest,
      children: [
        iconAlignment !== "right" && iconEl,
        showText && /* @__PURE__ */ jsx("span", { className: cx("pointer-events-none select-none", styles2.content), children: text || children }),
        iconAlignment === "right" && iconEl
      ]
    }
  );
};
var AlertDialog = ({ type, icon, title, message, onOk, onClose, hideModal }) => {
  const handleConfirm = () => {
    if (onOk) onOk();
    if (onClose) onClose();
    if (hideModal) hideModal();
  };
  return /* @__PURE__ */ jsxs(Fragment, { children: [
    /* @__PURE__ */ jsxs("div", { className: "flex items-start p-4", children: [
      icon && /* @__PURE__ */ jsx(Icon, { name: icon, type, size: "l", classes: "mr-4 mt-1 shrink-0" }),
      /* @__PURE__ */ jsxs("div", { className: "grow", children: [
        title && /* @__PURE__ */ jsx("h2", { className: "text-2xl font-semibold text-strong", children: title }),
        /* @__PURE__ */ jsx("p", { className: "text-sm text-muted leading-relaxed", children: message })
      ] })
    ] }),
    /* @__PURE__ */ jsx("div", { className: "flex justify-end p-4", children: /* @__PURE__ */ jsx(Button, { onClick: handleConfirm, children: "Ok" }) })
  ] });
};
var alert_dialog_default = AlertDialog;
var AlertContext = createContext({
  showAlert: (_props) => {
  },
  showError: (_message) => {
  }
});
var AlertProvider = ({ children }) => {
  const hideModal = modalService.hide.bind(modalService);
  const showAlert = (props) => {
    modalService.show(AlertDialog, {
      type: props?.type,
      icon: props?.icon,
      title: props?.title,
      message: props?.message,
      onOk: props?.onOk,
      onClose: hideModal,
      hideModal
    });
  };
  const showError = (message) => {
    modalService.show(AlertDialog, {
      type: "error",
      icon: "warning",
      title: "Error",
      message,
      onClose: hideModal
    });
  };
  useEffect(() => {
    const handleGlobalAlert = (options) => {
      modalService.show(AlertDialog, {
        type: options.type,
        icon: options.icon,
        title: options.title,
        message: options.message,
        onOk: options.onOk,
        onClose: hideModal,
        hideModal
      });
    };
    alertService.register(handleGlobalAlert);
    return () => {
      alertService.unregister(handleGlobalAlert);
    };
  }, []);
  return /* @__PURE__ */ jsx(AlertContext.Provider, { value: { showAlert, showError }, children });
};
var useAlert = () => {
  return useContext(AlertContext);
};

export { AlertProvider, alert_dialog_default as default, useAlert };
//# sourceMappingURL=alert-dialog.js.map
//# sourceMappingURL=alert-dialog.js.map