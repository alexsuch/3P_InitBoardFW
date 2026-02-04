import * as preact from 'preact';
import { InternalToastOptions } from '../toast-service.js';

interface ToastProps {
    options: InternalToastOptions;
    onClose: () => void;
}
declare const Toast: ({ options, onClose }: ToastProps) => preact.JSX.Element;

export { Toast, type ToastProps };
