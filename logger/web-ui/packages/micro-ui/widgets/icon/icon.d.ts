import * as preact from 'preact';
import { CSSProperties } from 'preact/compat';

type IconType = 'info' | 'success' | 'warning' | 'error' | 'unknown';
interface IconProps {
    name: string;
    size?: 'xs' | 's' | 'm' | 'l' | 'xl';
    subscript?: string;
    onClick?: () => void;
    type?: IconType;
    isDisabled?: boolean;
    classes?: string;
    style?: CSSProperties;
}
declare const Icon: (props: IconProps) => preact.JSX.Element;

export { Icon, type IconProps, type IconType };
