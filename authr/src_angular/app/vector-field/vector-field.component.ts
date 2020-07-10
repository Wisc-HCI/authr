import {FocusMonitor} from '@angular/cdk/a11y';
import {coerceBooleanProperty} from '@angular/cdk/coercion';
import {Component, ElementRef, Input, Output, OnDestroy, EventEmitter} from '@angular/core';
import {MatFormFieldControl} from '@angular/material';
import {Subject} from 'rxjs';

@Component({
  selector: 'vector-field',
  templateUrl: 'vector-field.component.html',
  styleUrls: ['vector-field.component.css'],
})
export class VectorFieldComponent {}

export class Vector {
  constructor(public x: number, public y: number, public z: number) {}
}

@Component({
  selector: 'vector-input',
  templateUrl: 'vector-field-input.html',
  styleUrls: ['vector-field-input.css'],
  providers: [{provide: MatFormFieldControl, useExisting: VectorInput}],
  host: {
    '[class.example-floating]': 'shouldLabelFloat',
    '[id]': 'id',
    '[attr.aria-describedby]': 'describedBy',
  }
})
export class VectorInput implements MatFormFieldControl<Vector>, OnDestroy {
  static nextId = 0;

  stateChanges = new Subject<void>();
  focused = false;
  ngControl = null;
  errorState = false;
  controlType = 'vector-input';
  id = `vector-input-${VectorInput.nextId++}`;
  describedBy = '';

  get empty() {
    const {x, y, z} = this._value;
    return (!x && x != 0) && (!y && y != 0) && (!z && z != 0);
  }

  get shouldLabelFloat() { return this.focused || !this.empty; }

  @Input()
  get placeholder(): string { return this._placeholder; }
  set placeholder(value: string) {
    this._placeholder = value;
    this.stateChanges.next();
  }
  public _placeholder: string;

  @Input()
  get required(): boolean { return this._required; }
  set required(value: boolean) {
    this._required = coerceBooleanProperty(value);
    this.stateChanges.next();
  }
  public _required = false;

  @Input()
  get disabled(): boolean { return this._disabled; }
  set disabled(value: boolean) {
    this._disabled = coerceBooleanProperty(value);
    this.stateChanges.next();
  }
  public _disabled = false;

  @Output() onChange: EventEmitter<any> = new EventEmitter();

  @Input()
  get value(): Vector {
    return this._value;
  }
  set value(value: Vector | null) {
    this._value = value || new Vector(null, null, null);
    this.stateChanges.next();
  }
  private _value = new Vector(null, null, null);

  constructor(public fm: FocusMonitor, public elRef: ElementRef<HTMLElement>) {

    fm.monitor(elRef, true).subscribe(origin => {
      this.focused = !!origin;
      this.stateChanges.next();
    });
  }

  ngOnDestroy() {
    this.stateChanges.complete();
    this.fm.stopMonitoring(this.elRef);
  }

  setDescribedByIds(ids: string[]) {
    this.describedBy = ids.join(' ');
  }

  onContainerClick(event: MouseEvent) {
    if ((event.target as Element).tagName.toLowerCase() != 'input') {
      this.elRef.nativeElement.querySelector('input')!.focus();
    }
  }

  modelChanged(obj,property) {
    console.log("model changed");
    console.log(obj,property);
    if (obj === null || obj === undefined) obj = 0;
    this._value[property] = obj;
    console.log("emit");
    this.onChange.emit(this._value);
    this.stateChanges.next();

  }
}
