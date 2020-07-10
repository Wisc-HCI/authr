import { Component, OnInit, Input } from '@angular/core';
import { AuthrService }             from '../services/authr.service';
import { DragulaService }           from 'ng2-dragula'

@Component({
  selector: 'app-elements-sidebar',
  templateUrl: './elements-sidebar.component.html',
  styleUrls: ['./elements-sidebar.component.css'],
  host: {'class': 'content-flex'}
})
export class ElementsSidebarComponent implements OnInit {

      constructor(public authrService: AuthrService,public dragulaService: DragulaService) {}

      ngOnInit() {}
  }
