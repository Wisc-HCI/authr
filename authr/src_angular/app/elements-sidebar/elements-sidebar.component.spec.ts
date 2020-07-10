import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { ElementsSidebarComponent } from './elements-sidebar.component';

describe('ElementsSidebarComponent', () => {
  let component: ElementsSidebarComponent;
  let fixture: ComponentFixture<ElementsSidebarComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ ElementsSidebarComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(ElementsSidebarComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
