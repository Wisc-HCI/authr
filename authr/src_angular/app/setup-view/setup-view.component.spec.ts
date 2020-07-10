import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { SetupViewComponent } from './setup-view.component';

describe('SetupViewComponent', () => {
  let component: SetupViewComponent;
  let fixture: ComponentFixture<SetupViewComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ SetupViewComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(SetupViewComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
