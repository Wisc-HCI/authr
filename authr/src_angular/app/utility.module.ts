import { NgModule, Pipe, PipeTransform, Input, Directive, ElementRef } from '@angular/core';

@Pipe({name: 'replaceUnderscores'})
export class ReplaceUnderscoresPipe implements PipeTransform {
  transform(value: string): string {
    return value.replace('_',' ');
  }
}

export function enumSelector(definition) {
    var enumSelect = [];
    for (let key of Object.keys(definition)) {
        if (definition[key] !== 'notset' && definition[key] !== 'opt') {
            enumSelect.push({ value: definition[key], title: key })
        }
    }
    return enumSelect;
}

export function rgbToHex(r, g, b) {
    return "#" + ((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1);
}

export function hexToRgb(hex) {
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}

@Directive({
  selector: '[matBadgeIcon]'
})
export class MatBadgeIconDirective {

  @Input() matBadgeIcon: string;

  constructor(private el: ElementRef) {}

  ngOnInit() {
    const badge = this.el.nativeElement.querySelector('.mat-badge-content');
    if (this.matBadgeIcon) {
        badge.style.display = 'flex';
        badge.style.alignItems = 'center';
        badge.style.justifyContent = 'center';
        badge.innerHTML = `<i class="material-icons" style="font-size: 10px; color:white">${this.matBadgeIcon}</i>`;
    }
  }
}


@NgModule({
  exports: [ReplaceUnderscoresPipe],
  declarations: [ReplaceUnderscoresPipe],
  providers: [ReplaceUnderscoresPipe]
})
export class UtilityModule { }
