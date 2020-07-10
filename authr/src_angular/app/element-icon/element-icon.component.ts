import { Component,
         OnInit,
         OnChanges,
         Input, SimpleChanges }  from '@angular/core';
import { AuthrService }          from '../services/authr.service';

@Component({
  selector: 'element-icon',
  templateUrl: './element-icon.component.html',
  styleUrls: ['./element-icon.component.scss']
})
export class ElementIconComponent {

  @Input() size: string = 'large';
  @Input() type: string;
  @Input() element: any;

  constructor(public authrService:AuthrService) {}

}
