import { Signal } from '@preact/signals';

interface IInputProps {
    onValueChange?: (value: any) => void;
    valueSignal?: Signal<any>;
    pasteConverter?: (value: string) => string | number;
}

export type { IInputProps as I };
