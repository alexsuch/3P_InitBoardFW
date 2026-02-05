import { ComponentType } from 'preact';

type ToastType = 'info' | 'success' | 'warning' | 'error';
type ToastPosition = 'top-center' | 'bottom-center';
interface ToastOptions {
    id?: string;
    message?: string;
    title?: string;
    type?: ToastType;
    icon?: string;
    position?: ToastPosition;
    duration?: number | null;
    component?: ComponentType<any>;
    props?: any;
    onClick?: () => void;
}
type InternalToastOptions = Required<Pick<ToastOptions, 'id'>> & ToastOptions;
type ShowToastFunction = (options: ToastOptions) => string;
type HideToastFunction = (id: string) => void;
declare class ToastService {
    private showToastFunction;
    private hideToastFunction;
    register(showFn: ShowToastFunction, hideFn: HideToastFunction): void;
    show(options: ToastOptions): string | undefined;
    hide(id: string): void;
    showError(message: string, title?: string): string | undefined;
    showInfo(message: string, title?: string): string | undefined;
    showSuccess(message: string, title?: string): string | undefined;
    showWarning(message: string, title?: string): string | undefined;
}
declare const toastService: ToastService;

export { type InternalToastOptions, type ToastOptions, type ToastPosition, type ToastType, toastService };
