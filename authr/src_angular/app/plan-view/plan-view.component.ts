import { Component, ViewEncapsulation, OnInit } from '@angular/core';
import { AuthrService } from '../services/authr.service';

@Component({
  selector: 'app-plan-view',
  templateUrl: './plan-view.component.html',
  styleUrls: ['./plan-view.component.css'],
  host: {'class': 'content-flex'},
  encapsulation: ViewEncapsulation.None
})
export class PlanViewComponent implements OnInit {

  constructor(public authrService:AuthrService) {
    this.authrService.workspace_split=100;
  }

  ngOnInit() {
    this.authrService.focused = null;
  }
}
