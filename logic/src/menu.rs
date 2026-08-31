//! Menu model (SPECS §9.4). Rendering and button handling live in the firmware UI task.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MenuItem {
    Fps,
    Exposure,
    Roll,
    Track,
    Boost,
    Transport,
    Settings,
    About,
}

pub const MENU_LEN: usize = 8;

const ALL: [MenuItem; MENU_LEN] = [
    MenuItem::Fps,
    MenuItem::Exposure,
    MenuItem::Roll,
    MenuItem::Track,
    MenuItem::Boost,
    MenuItem::Transport,
    MenuItem::Settings,
    MenuItem::About,
];

impl MenuItem {
    pub const fn from_index(i: usize) -> Self {
        ALL[i % MENU_LEN]
    }

    pub const fn title(self) -> &'static str {
        match self {
            MenuItem::Fps => "FPS",
            MenuItem::Exposure => "EXPOSURE",
            MenuItem::Roll => "ROLL",
            MenuItem::Track => "TRACK",
            MenuItem::Boost => "BOOST",
            MenuItem::Transport => "TRANSPORT",
            MenuItem::Settings => "SETTINGS",
            MenuItem::About => "ABOUT",
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MenuState {
    selected: usize,
}

impl MenuState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn current(&self) -> MenuItem {
        MenuItem::from_index(self.selected)
    }

    pub fn next(&mut self) {
        self.selected = (self.selected + 1) % MENU_LEN;
    }

    pub fn prev(&mut self) {
        self.selected = (self.selected + MENU_LEN - 1) % MENU_LEN;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn navigation_wraps() {
        let mut m = MenuState::new();
        assert_eq!(m.current(), MenuItem::Fps);
        m.prev();
        assert_eq!(m.current(), MenuItem::About);
        for _ in 0..MENU_LEN {
            m.next();
        }
        assert_eq!(m.current(), MenuItem::About);
    }
}
