import { Component, ViewEncapsulation, Inject } from '@angular/core';
import { fadeAnimation } from './animations';
import { MatIconRegistry } from "@angular/material/icon";
import { DomSanitizer } from "@angular/platform-browser";
import { PLATFORM_ID } from '@angular/core';
import { isPlatformBrowser, isPlatformServer } from '@angular/common';

// Service
import { AuthrService } from './services/authr.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css'],
  animations: [fadeAnimation],
  encapsulation: ViewEncapsulation.None
})
export class AppComponent {
  title = 'Authr';

  navLinks = [
      {label: 'Setup',      path: 'setup',      enabled:true},
      {label: 'Plan',       path: 'plan',       enabled:true},
      {label: 'Simulate',   path: 'simulate',   enabled:true},
  ]

  constructor(public authrService: AuthrService,
              private matIconRegistry: MatIconRegistry,
              private domSanitizer: DomSanitizer,
              @Inject(PLATFORM_ID) private platformId: string) {

                  let icons:string[] = ['robot','human','opt','cube','sphere','cylinder','container',
                                        'loc-robot','loc-human','loc-cube','loc-sphere','loc-cylinder','loc-container',
                                        'notset','authr','logo']
                  const svgBaseUrl = 'assets/img/';
                  const domain = (isPlatformServer(platformId)) ? 'http://localhost:4200/' : '';

                  icons.forEach(icon => {
                      this.matIconRegistry.addSvgIcon(icon, this.domSanitizer.bypassSecurityTrustResourceUrl(domain + svgBaseUrl+icon.replace("-","_")+".svg"));
                  })
  }
}
