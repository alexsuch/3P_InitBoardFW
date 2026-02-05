import * as preact from 'preact';

interface ModalProps {
    isOpen: boolean;
    onClose: () => void;
    title?: string;
    children?: any;
}
declare const Modal: ({ isOpen, onClose, children }: ModalProps) => preact.JSX.Element | null;

interface ModalContextProps {
    showModal: (Component: any, props?: any) => void;
    hideModal: () => void;
}
declare const ModalProvider: ({ children }: {
    children: any;
}) => preact.JSX.Element;
declare const useModal: () => ModalContextProps;

export { ModalProvider, Modal as default, useModal };
