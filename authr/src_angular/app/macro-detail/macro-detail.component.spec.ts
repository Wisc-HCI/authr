import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { MacroDetailComponent } from './macro-detail.component';

describe('MacroDetailComponent', () => {
  let component: MacroDetailComponent;
  let fixture: ComponentFixture<MacroDetailComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ MacroDetailComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(MacroDetailComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
