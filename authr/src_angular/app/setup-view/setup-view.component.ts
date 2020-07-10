import { Component, OnInit } from '@angular/core';
import { AuthrService } from '../services/authr.service';

@Component({
  selector: 'app-setup-view',
  templateUrl: './setup-view.component.html',
  styleUrls: ['./setup-view.component.scss'],
  host: {'class': 'content-flex'}
})
export class SetupViewComponent implements OnInit {

  public navLinks = [
        {label: 'General',      path: 'general'},
        {label: 'Agents',       path: 'agents'},
        {label: 'Things',       path: 'things'},
        {label: 'Destinations', path: 'destinations'}
  ]

  constructor(private authrService:AuthrService) {
    this.authrService.workspace_split=60;
  }

  ngOnInit() {
    this.authrService.markerVisibilityTopic.publish({"data":"interactive"});
  }









}
