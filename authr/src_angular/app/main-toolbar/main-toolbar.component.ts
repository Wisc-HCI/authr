import { Component, OnInit, OnDestroy } from '@angular/core';
import { AuthrService } from "../services/authr.service";

@Component({
  selector: 'app-main-toolbar',
  templateUrl: './main-toolbar.component.html',
  styleUrls: ['./main-toolbar.component.css']
})
export class MainToolbarComponent {

  public title:string = 'Authr';

  constructor(public authrService: AuthrService) {}

}
