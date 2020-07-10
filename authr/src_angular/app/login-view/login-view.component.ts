import { Component, OnInit } from '@angular/core';
import { AuthrService } from '../services/authr.service';
import { Router } from '@angular/router';

@Component({
  selector: 'app-login-view',
  templateUrl: './login-view.component.html',
  styleUrls: ['./login-view.component.scss'],
  host: {'class': 'content-flex'}
})
export class LoginViewComponent implements OnInit {

  public interval;

  constructor(public authrService:AuthrService,
              public router:Router) {
                this.authrService.workspace_split=0;
              }

  ngOnInit() {
      this.interval = setInterval(() => {
          // Do your update stuff...
          if (this.authrService.taskKeys.length == 0 && this.authrService.timeweight == 0 && this.loading) {
              this.authrService.requestCacheRefresh.publish({data:'{}'});
          } else if (this.loading) {
              this.authrService.loading = false;
              console.log("Loaded");
              this.router.navigate(['/setup',{ outlets: { setup: ['general'] } }]);
          }
      }, 1500)
  }

  public set loading(value:boolean) {
      if (this.authrService.loading && !value) {
          clearTimeout(this.interval);
      }
      this.authrService.loading = value;
  }

  public get loading() {
      return this.authrService.loading;
  }

  private connect() {
    this.authrService.connect(this.authrService.address).subscribe(success => {
      if (success) {this.loading = true;}
    })
  }
}
