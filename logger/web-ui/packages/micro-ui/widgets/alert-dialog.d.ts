import * as preact from 'preact';

interface AlertDialogProps {
    type?: 'info' | 'success' | 'warning' | 'error';
    icon?: string;
    title?: string;
    message?: string;
    onOk?: () => void;
    onClose?: () => void;
    hideModal?: () => void;
}
declare const AlertDialog: ({ type, icon, title, message, onOk, onClose, hideModal }: AlertDialogProps) => preact.JSX.Element;

interface AlertContextProps {
    showAlert: (props?: AlertDialogProps) => void;
    showError: (message: string) => void;
}
declare const AlertProvider: ({ children }: {
    children: any;
}) => preact.JSX.Element;
declare const useAlert: () => AlertContextProps;

export { AlertProvider, AlertDialog as default, useAlert };
