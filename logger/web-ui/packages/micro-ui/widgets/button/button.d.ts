import * as preact from 'preact';
import { ButtonHTMLAttributes } from 'preact/compat';

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
    variant?: 'filled' | 'outlined' | 'text' | 'ghost' | 'elevated';
    color?: 'primary' | 'secondary' | 'warning' | 'success' | 'error' | 'inherit';
    iconOnly?: boolean;
    iconAlignment?: 'left' | 'center' | 'right';
    size?: 'xs' | 's' | 'm' | 'l' | 'xl';
    iconName?: string;
    iconSize?: 'xs' | 's' | 'm' | 'l' | 'xl';
    iconClassName?: string;
    className?: string;
    children?: any;
    text?: string;
    onClick?: (e: any) => void;
    onMouseDown?: (e: any) => void;
    onMouseUp?: (e: any) => void;
    onPointerDown?: (e: any) => void;
    id?: string;
    isDisabled?: boolean;
}
declare const Button: ({ variant, color, iconOnly, iconAlignment, size, iconName, iconSize, iconClassName, className, children, text, onClick, onMouseDown, onMouseUp, onPointerDown, id, disabled, isDisabled, ...rest }: ButtonProps) => preact.JSX.Element;

export { Button, type ButtonProps };
