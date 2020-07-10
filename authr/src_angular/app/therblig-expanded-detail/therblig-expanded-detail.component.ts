import { Component, OnInit, Input } from '@angular/core';
import { AuthrService } from '../services/authr.service';

@Component({
  selector: 'app-therblig-expanded-detail',
  templateUrl: './therblig-expanded-detail.component.html',
  styleUrls: ['./therblig-expanded-detail.component.scss'],
  host: {'class': 'content-flex'}
})
export class TherbligExpandedDetailComponent implements OnInit {

  @Input() therblig: any;

  constructor(public authrService: AuthrService) {}

  ngOnInit() {}

}
