import { createContext } from 'preact';
import { useRef, useEffect, useContext } from 'preact/hooks';
import { useSignal } from '@preact/signals';
import styles from '../../modal.module-SN5JSFRU.module.css';
import { modalService } from 'micro-ui/widgets/modal-service';
import { jsx, jsxs } from 'preact/jsx-runtime';

// src/widgets/modal/modal.tsx

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
var Modal = ({ isOpen, onClose, children }) => {
  const isVisible = useSignal(isOpen);
  const bdRef = useRef(null);
  const isMouseDownOnBackdrop = useSignal(false);
  const isMouseDownOnContent = useSignal(false);
  useEffect(() => {
    if (isOpen) {
      isVisible.value = true;
    } else {
      setTimeout(() => isVisible.value = false, 300);
    }
  }, [isOpen]);
  const handleBackdropMouseDown = (e) => {
    if (e.target === bdRef.current) {
      isMouseDownOnBackdrop.value = true;
      isMouseDownOnContent.value = false;
    }
  };
  const handleBackdropMouseUp = (e) => {
    if (isMouseDownOnBackdrop.value && e.target === bdRef.current && !isMouseDownOnContent.value) {
      onClose();
    }
    isMouseDownOnBackdrop.value = false;
    isMouseDownOnContent.value = false;
  };
  const handleContentMouseDown = () => {
    isMouseDownOnContent.value = true;
    isMouseDownOnBackdrop.value = false;
  };
  const handleContentMouseUp = () => {
    isMouseDownOnContent.value = false;
  };
  const handleBackdropTouchStart = (e) => {
    if (e.target === bdRef.current) {
      isMouseDownOnBackdrop.value = true;
      isMouseDownOnContent.value = false;
    }
  };
  const handleBackdropTouchEnd = (e) => {
    if (isMouseDownOnBackdrop.value && e.target === bdRef.current && !isMouseDownOnContent.value) {
      onClose();
    }
    isMouseDownOnBackdrop.value = false;
    isMouseDownOnContent.value = false;
  };
  const handleContentTouchStart = () => {
    isMouseDownOnContent.value = true;
    isMouseDownOnBackdrop.value = false;
  };
  const handleContentTouchEnd = () => {
    isMouseDownOnContent.value = false;
  };
  if (!isVisible.value) return null;
  return /* @__PURE__ */ jsx(
    "div",
    {
      ref: bdRef,
      className: cx(styles.b, { [styles.open]: isOpen }),
      "data-testid": "modal-backdrop",
      onMouseDown: handleBackdropMouseDown,
      onMouseUp: handleBackdropMouseUp,
      onTouchStart: handleBackdropTouchStart,
      onTouchEnd: handleBackdropTouchEnd,
      children: /* @__PURE__ */ jsxs(
        "div",
        {
          className: styles.c,
          onMouseDown: handleContentMouseDown,
          onMouseUp: handleContentMouseUp,
          onTouchStart: handleContentTouchStart,
          onTouchEnd: handleContentTouchEnd,
          children: [
            /* @__PURE__ */ jsx("button", { className: styles.x, onClick: onClose, children: "\xD7" }),
            /* @__PURE__ */ jsx("div", { className: styles.mb, children })
          ]
        }
      )
    }
  );
};
var modal_default = Modal;
var ModalContext = createContext({
  showModal: () => {
  },
  hideModal: () => {
  }
});
var ModalProvider = ({ children }) => {
  const modalContent = useSignal(null);
  const modalProps = useSignal({});
  const isOpen = useSignal(false);
  const scrollLockRef = useRef(null);
  const registeredRef = useRef(false);
  const restoreScroll = () => {
    const lockState = scrollLockRef.current;
    if (!lockState) {
      return;
    }
    lockState.element.style.overflow = lockState.overflow;
    lockState.element.style.paddingRight = lockState.paddingRight;
    scrollLockRef.current = null;
  };
  const lockScroll = () => {
    if (typeof document === "undefined") {
      return;
    }
    const scrollingElement = document.scrollingElement ?? document.body;
    if (!scrollingElement || scrollLockRef.current) {
      return;
    }
    const currentOverflow = scrollingElement.style.overflow;
    const currentPadding = scrollingElement.style.paddingRight;
    const scrollbarWidth = typeof window !== "undefined" ? window.innerWidth - document.documentElement.clientWidth : 0;
    scrollLockRef.current = {
      element: scrollingElement,
      overflow: currentOverflow,
      paddingRight: currentPadding
    };
    const computedStyle = typeof window !== "undefined" ? window.getComputedStyle(scrollingElement) : null;
    const currentPaddingValue = computedStyle ? parseInt(computedStyle.paddingRight, 10) || 0 : 0;
    scrollingElement.style.overflow = "hidden";
    if (scrollbarWidth > 0) {
      scrollingElement.style.paddingRight = `${currentPaddingValue + scrollbarWidth}px`;
    }
  };
  const hideModal = () => {
    restoreScroll();
    isOpen.value = false;
  };
  const showModal = (Component, props = {}) => {
    lockScroll();
    modalContent.value = Component;
    modalProps.value = { ...props, hideModal };
    isOpen.value = true;
  };
  if (!registeredRef.current) {
    console.log("registering modal service");
    modalService.register(showModal, hideModal);
    registeredRef.current = true;
  }
  useEffect(() => {
    return () => {
      restoreScroll();
    };
  }, []);
  const renderModalContent = () => {
    if (!isOpen.value || !modalContent.value) return null;
    const Component = modalContent.value;
    const props = modalProps.value;
    return /* @__PURE__ */ jsx(Component, { ...props });
  };
  return /* @__PURE__ */ jsxs(ModalContext.Provider, { value: { showModal, hideModal }, children: [
    children,
    isOpen.value && /* @__PURE__ */ jsx(Modal, { isOpen: isOpen.value, onClose: hideModal, children: modalContent.value ? renderModalContent() : /* @__PURE__ */ jsx("div", { children: "Loading..." }) })
  ] });
};
var useModal = () => {
  return useContext(ModalContext);
};

export { ModalProvider, modal_default as default, useModal };
//# sourceMappingURL=modal.js.map
//# sourceMappingURL=modal.js.map