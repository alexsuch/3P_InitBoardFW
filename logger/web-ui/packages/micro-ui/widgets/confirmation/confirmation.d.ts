import * as preact from 'preact';

interface ConfirmationProps {
    title?: string;
    message?: string;
    onConfirm: (result: boolean) => void;
    onClose?: () => void;
    hideModal: () => void;
    iconName?: string;
    iconType?: 'info' | 'success' | 'warning' | 'error';
    confirmText?: string;
    cancelText?: string;
}
declare const Confirmation: ({ title, message, onConfirm, onClose, hideModal, iconName, iconType, confirmText, cancelText, }: ConfirmationProps) => preact.JSX.Element;

export { Confirmation, type ConfirmationProps, Confirmation as default };
